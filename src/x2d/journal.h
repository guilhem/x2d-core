#pragma once

// Durable shutter lifecycle journal. The 64 KiB region is unchanged: two
// 32 KiB banks, 42 append-only snapshots/bank. Each snapshot has a 512-byte
// CRC-protected body and a separately verified 256-byte commit. The final
// 512 bytes of each bank hold a downgrade fence: v1 readers must fail closed,
// rather than mistake a v2-only bank for uninitialized storage.
//
// Logical IDs and RF prefixes are never recycled. Sixteen physical slots are
// reusable; replacement changes a private incarnation, not the logical ID.
// Counters are consumed before RF. Torn intact bodies conservatively burn
// allocations/counters; they never automatically enable a binding. No RF job,
// attempt permit, or motor position is restored. Unknown storage is not erased.
// Mutation calls never rotate banks. maintain() runs at idle; 16 snapshot
// records and counter 0xFFFF are reserved for critical STOP reservations.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace x2d {
namespace journal {

constexpr uint8_t SLOTS = 16;
constexpr uint32_t PAGE_BYTES = 256;
constexpr uint32_t BODY_BYTES = 2 * PAGE_BYTES;
constexpr uint32_t RECORD_BYTES = BODY_BYTES + PAGE_BYTES;
constexpr uint32_t SECTOR_BYTES = 4096;
constexpr uint32_t BANK_BYTES = 32768;
constexpr uint32_t REGION_BYTES = 2 * BANK_BYTES;
constexpr uint32_t PAIRS_PER_BANK = BANK_BYTES / RECORD_BYTES;
constexpr uint32_t SECTORS_PER_BANK = BANK_BYTES / SECTOR_BYTES;
constexpr uint32_t FENCE_AT = PAIRS_PER_BANK * RECORD_BYTES;
constexpr uint32_t CRITICAL_PAIRS = SLOTS;
constexpr uint32_t MAINTAIN_BELOW = CRITICAL_PAIRS + 16;
constexpr uint32_t MAX_NEXT = 0x10000;
constexpr uint32_t RF_PREFIXES = 0x10000;
constexpr uint8_t MAX_EXCLUSIONS = 32; // 16 v1 identities + trusted primitive inputs
static_assert(FENCE_AT + 2 * PAGE_BYTES == BANK_BYTES, "bank fence does not fit");
static_assert(MAINTAIN_BELOW < PAIRS_PER_BANK, "bank too small for STOP reserve");

// Flash backend for exactly REGION_BYTES. The parent maps offset 0 to the
// start of the reserved region and must keep it outside the sketch and outside
// any filesystem it mounts. Writes happen only in erased space.
class Flash {
 public:
  virtual ~Flash() = default;
  virtual uint32_t size() const = 0;  // must equal REGION_BYTES
  virtual bool read(uint32_t offset, void* out, uint32_t length) = 0;
  virtual bool erase_sector(uint32_t offset) = 0;  // SECTOR_BYTES aligned
  virtual bool program_page(uint32_t offset, const uint8_t* page) = 0;  // PAGE_BYTES aligned
};

// Host-test backend with strict NOR rules and power-cut injection.
class MemoryFlash final : public Flash {
 public:
  MemoryFlash() { memset(data_, 0xFF, sizeof(data_)); }
  uint32_t size() const override { return REGION_BYTES; }
  bool read(uint32_t offset, void* out, uint32_t length) override {
    if (!powered_ || offset > REGION_BYTES || length > REGION_BYTES - offset)
      return false;
    memcpy(out, data_ + offset, length);
    return true;
  }
  bool erase_sector(uint32_t offset) override {
    if (!powered_ || offset % SECTOR_BYTES || offset >= REGION_BYTES) return false;
    if (cut_now()) {
      const uint32_t done = torn_ < SECTOR_BYTES ? torn_ : SECTOR_BYTES;
      memset(data_ + offset, 0xFF, done);
      if (done < SECTOR_BYTES) data_[offset + done] |= 0xA5;
      return false;
    }
    memset(data_ + offset, 0xFF, SECTOR_BYTES);
    ++erases_;
    ++mutations_;
    return true;
  }
  bool program_page(uint32_t offset, const uint8_t* page) override {
    if (!powered_ || offset % PAGE_BYTES || offset >= REGION_BYTES) return false;
    for (uint32_t i = 0; i < PAGE_BYTES; ++i) {
      if (data_[offset + i] != 0xFF) {
        ++violations_;
        return false;
      }
    }
    if (cut_now()) {
      const uint32_t done = torn_ < PAGE_BYTES ? torn_ : PAGE_BYTES;
      memcpy(data_ + offset, page, done);
      if (done < PAGE_BYTES) data_[offset + done] = page[done] | 0x55;
      return false;
    }
    memcpy(data_ + offset, page, PAGE_BYTES);
    if (flip_next_) {
      data_[offset + 100] ^= 0x01;  // silent failure: returns true
      flip_next_ = false;
    }
    ++programs_;
    ++mutations_;
    return true;
  }

  // The (mutations+1)-th next erase/program tears after torn_bytes, then power
  // is lost until restore_power().
  void cut_after(uint32_t mutations, uint32_t torn_bytes) {
    cut_at_ = mutations_ + mutations;
    torn_ = torn_bytes;
    armed_ = true;
  }
  void restore_power() {
    powered_ = true;
    armed_ = false;
  }
  void flip_next_program() { flip_next_ = true; }
  bool powered() const { return powered_; }
  uint8_t* raw() { return data_; }
  uint32_t mutations() const { return mutations_; }
  uint32_t programs() const { return programs_; }
  uint32_t erases() const { return erases_; }
  uint32_t violations() const { return violations_; }

 private:
  bool cut_now() {
    if (!armed_ || mutations_ != cut_at_) return false;
    armed_ = false;
    powered_ = false;
    return true;
  }
  uint8_t data_[REGION_BYTES];
  bool powered_ = true, armed_ = false, flip_next_ = false;
  uint32_t mutations_ = 0, programs_ = 0, erases_ = 0, violations_ = 0;
  uint32_t cut_at_ = 0, torn_ = 0;
};

enum class StorageState : uint8_t { empty, ready, corrupt, full, legacy, initializing };
enum class SlotState : uint8_t { unused = 0, pending = 1, paired = 2, retired = 3 };
enum class Status : uint8_t {
  ok, unknown_slot, bad_argument, identity_in_use, exhausted, no_space,
  corrupt, io_error, not_initialized, busy, not_paired, disabled,
  no_candidate, no_attempt, retry_unavailable, incarnation_mismatch,
  identity_exhausted, logical_exhausted, inventory_full,
};
inline const char* state_name(StorageState state) {
  switch (state) {
    case StorageState::empty: return "empty";
    case StorageState::ready: return "ready";
    case StorageState::full: return "full";
    case StorageState::legacy: return "legacy";
    case StorageState::initializing: return "initializing";
    default: return "corrupt";
  }
}
inline const char* slot_state_name(SlotState state) {
  switch (state) {
    case SlotState::pending: return "pending";
    case SlotState::paired: return "paired";
    case SlotState::retired: return "retired";
    default: return nullptr;
  }
}
inline const char* protocol_error(Status status) {
  switch (status) {
    case Status::ok: return nullptr;
    case Status::unknown_slot: return "unknown_shutter";
    case Status::exhausted: return "counter_exhausted";
    case Status::no_space: return "storage_full";
    case Status::not_initialized: return "initialization_required";
    case Status::busy: return "association_pending";
    case Status::not_paired: return "not_paired";
    case Status::disabled: return "disabled";
    case Status::no_candidate: return "no_pending_association";
    case Status::no_attempt: return "no_association_attempt";
    case Status::retry_unavailable: return "retry_unavailable";
    case Status::incarnation_mismatch: return "stale_incarnation";
    case Status::identity_exhausted: return "identity_exhausted";
    case Status::logical_exhausted: return "logical_ids_exhausted";
    case Status::inventory_full: return "inventory_full";
    case Status::bad_argument:
    case Status::identity_in_use: return "invalid_request";
    default: return "storage_corrupt";
  }
}

struct Shutter {
  uint8_t shutter_id = 0;
  SlotState state = SlotState::unused;
  uint8_t last_command = 0;
  uint8_t logical_id = 0;
  bool in_service = false, has_candidate = false, replacement = false;
  uint8_t attempts = 0;
  uint32_t incarnation = 0, candidate_incarnation = 0;
};
struct NewController {
  uint32_t identity = 0;
  uint16_t first_counter = 0;
  uint64_t generation = 0;
};
struct Reservation {
  uint8_t shutter_id = 0;
  uint32_t identity = 0;
  uint16_t counter = 0;
  uint32_t incarnation = 0;
};

namespace detail {
constexpr uint32_t JOURNAL_MAGIC = 0x4A443258;
constexpr uint32_t COMMIT_MAGIC = 0x43443258;
constexpr uint32_t FENCE_MAGIC = 0x46443258;
constexpr uint16_t JOURNAL_VERSION = 2;
constexpr size_t SLOT_STRIDE = 16, EXCLUSIONS_AT = 56;
constexpr size_t SLOTS_AT = EXCLUSIONS_AT + MAX_EXCLUSIONS * 4;
constexpr size_t CRC_AT = BODY_BYTES - 4;
static_assert(SLOTS_AT + SLOTS * SLOT_STRIDE <= CRC_AT, "snapshot does not fit");
inline uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  while (length--) {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
inline void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) { for (int i=0; i<4; ++i) p[i]=uint8_t(v >> (8*i)); }
inline void put64(uint8_t* p, uint64_t v) { for (int i=0; i<8; ++i) p[i]=uint8_t(v >> (8*i)); }
inline uint16_t get16(const uint8_t* p) { return uint16_t(p[0] | (uint16_t(p[1]) << 8)); }
inline uint32_t get32(const uint8_t* p) { return uint32_t(p[0]) | (uint32_t(p[1])<<8) | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24); }
inline uint64_t get64(const uint8_t* p) { return get32(p) | (uint64_t(get32(p+4)) << 32); }
inline bool erased(const uint8_t* p, size_t size) { while (size--) if (*p++ != 0xFF) return false; return true; }
struct SlotRecord {
  uint32_t identity = 0, next = 0, incarnation = 0;
  uint8_t logical_id = 0, state = 0, command = 0;
  bool in_service = false;
};
struct Candidate {
  uint8_t slot = 0, attempts = 0;
  bool replacement = false;
  uint32_t identity = 0, next = 0, incarnation = 0;
};
struct Image {
  uint64_t generation = 0;
  uint32_t sequence = 0;
  uint16_t identity_seed = 0, next_logical = 2;
  uint8_t suffix = 1, exclusion_count = 0;
  bool initializing = false;
  uint32_t prefix_cursor = 0, next_incarnation = 1;
  Candidate candidate;
  uint32_t exclusions[MAX_EXCLUSIONS]{};
  SlotRecord slots[SLOTS];
};
inline bool excluded(const Image& image, uint32_t id) {
  for (uint8_t i=0; i<image.exclusion_count; ++i) if (image.exclusions[i]==id) return true;
  return false;
}
inline bool known_identity(const Image& image, uint32_t id) {
  if (!id || id>0xFFFFFF) return false;
  if (excluded(image,id)) return true;
  if (uint8_t(id) != image.suffix || !(id >> 8)) return false;
  return uint16_t(uint16_t(id >> 8) - image.identity_seed) < image.prefix_cursor;
}
inline bool valid_image(const Image& image) {
  if (!image.generation || !image.sequence || image.next_logical<2 || image.next_logical>255 ||
      image.prefix_cursor>RF_PREFIXES || !image.next_incarnation || image.exclusion_count>MAX_EXCLUSIONS)
    return false;
  for (uint8_t i=0; i<image.exclusion_count; ++i) {
    if (!image.exclusions[i] || image.exclusions[i]>0xFFFFFF) return false;
    for (uint8_t j=0; j<i; ++j) if (image.exclusions[j]==image.exclusions[i]) return false;
  }
  for (uint8_t i=0; i<SLOTS; ++i) {
    const auto& s=image.slots[i];
    if (s.state>uint8_t(SlotState::retired) || s.next>MAX_NEXT) return false;
    if (!s.state) {
      if (s.identity || s.next || s.incarnation || s.logical_id || s.command || s.in_service) return false;
      continue;
    }
    if (s.logical_id<2 || s.logical_id>=image.next_logical) return false;
    if (s.identity) {
      if (!known_identity(image,s.identity) || !s.incarnation || s.incarnation>=image.next_incarnation) return false;
    } else if (s.next || s.incarnation || s.command || s.state==uint8_t(SlotState::paired)) return false;
    if (s.in_service && s.state!=uint8_t(SlotState::paired)) return false;
    if (image.initializing) return false;
    for (uint8_t j=0; j<i; ++j) {
      if (image.slots[j].state && image.slots[j].logical_id==s.logical_id) return false;
      if (s.identity && image.slots[j].identity==s.identity) return false;
      if (s.incarnation && image.slots[j].incarnation==s.incarnation) return false;
    }
  }
  const auto& c=image.candidate;
  if (!c.slot) return !c.identity && !c.next && !c.incarnation && !c.attempts && !c.replacement;
  if (image.initializing || c.slot>SLOTS || !known_identity(image,c.identity) || !c.incarnation ||
      c.incarnation>=image.next_incarnation || c.attempts>2 || c.next>2u*c.attempts) return false;
  if (c.attempts==2 && c.next<2) return false;
  const auto& s=image.slots[c.slot-1];
  if (s.in_service || (c.replacement ? s.state!=uint8_t(SlotState::paired) :
      s.state!=uint8_t(SlotState::pending) || s.identity)) return false;
  for (const auto& slot : image.slots)
    if (slot.identity==c.identity || slot.incarnation==c.incarnation) return false;
  return true;
}
inline bool valid_transition(const Image& before, const Image& after) {
  if (before.generation!=after.generation || before.identity_seed!=after.identity_seed || before.suffix!=after.suffix ||
      after.next_logical<before.next_logical || after.next_incarnation<before.next_incarnation ||
      after.prefix_cursor<before.prefix_cursor || after.exclusion_count<before.exclusion_count ||
      (!before.initializing && after.initializing)) return false;
  for (uint8_t i=0; i<before.exclusion_count; ++i) if (before.exclusions[i]!=after.exclusions[i]) return false;
  for (uint8_t i=0; i<SLOTS; ++i) {
    const auto& p=before.slots[i]; const auto& n=after.slots[i];
    if (!p.state) {
      if (n.state && n.logical_id<before.next_logical) return false;
      continue;
    }
    if (!n.state) {
      if (p.state!=uint8_t(SlotState::pending) || p.identity) return false;
      continue;
    }
    if (n.logical_id!=p.logical_id) {
      if (p.state!=uint8_t(SlotState::retired) || n.logical_id<before.next_logical) return false;
      continue;
    }
    if (n.incarnation==p.incarnation) {
      if (n.identity!=p.identity || n.next<p.next || (p.state==uint8_t(SlotState::retired) && n.state!=p.state)) return false;
    } else {
      const auto& c=before.candidate;
      if (c.slot!=i+1 || n.incarnation!=c.incarnation || n.identity!=c.identity || n.next<c.next ||
          n.state!=uint8_t(SlotState::paired)) return false;
    }
  }
  const auto& p=before.candidate; const auto& n=after.candidate;
  if (n.slot && p.slot && n.incarnation==p.incarnation) {
    if (n.slot!=p.slot || n.identity!=p.identity || n.replacement!=p.replacement || n.next<p.next || n.attempts<p.attempts) return false;
  } else if (n.slot && n.incarnation<before.next_incarnation) return false;
  return true;
}
inline void encode_body(const Image& image, uint8_t* body) {
  memset(body,0xFF,BODY_BYTES);
  put32(body,JOURNAL_MAGIC); put16(body+4,JOURNAL_VERSION); put16(body+6,SLOTS);
  put64(body+8,image.generation); put32(body+16,image.sequence);
  put32(body+20,image.next_incarnation); put16(body+24,image.identity_seed);
  body[26]=image.suffix; body[27]=image.initializing ? 1 : 0;
  put32(body+28,image.prefix_cursor); put16(body+32,image.next_logical);
  body[34]=image.exclusion_count; body[35]=0;
  body[36]=image.candidate.slot; body[37]=image.candidate.attempts; body[38]=image.candidate.replacement ? 1 : 0; body[39]=0;
  put32(body+40,image.candidate.identity); put32(body+44,image.candidate.next); put32(body+48,image.candidate.incarnation); put32(body+52,0);
  for (uint8_t i=0; i<MAX_EXCLUSIONS; ++i) put32(body+EXCLUSIONS_AT+4*i,image.exclusions[i]);
  for (uint8_t i=0; i<SLOTS; ++i) {
    auto* s=body+SLOTS_AT+i*SLOT_STRIDE; const auto& v=image.slots[i];
    put32(s,v.identity); put32(s+4,v.next); put32(s+8,v.incarnation);
    s[12]=v.logical_id; s[13]=v.state; s[14]=v.command; s[15]=v.in_service ? 1 : 0;
  }
  put32(body+CRC_AT,crc32(body,CRC_AT));
}
inline bool valid_body(const uint8_t* body, Image* out) {
  if (get32(body)!=JOURNAL_MAGIC || get16(body+4)!=JOURNAL_VERSION || get16(body+6)!=SLOTS ||
      get32(body+CRC_AT)!=crc32(body,CRC_AT) || body[27]>1 || body[38]>1) return false;
  *out=Image{}; out->generation=get64(body+8); out->sequence=get32(body+16);
  out->next_incarnation=get32(body+20); out->identity_seed=get16(body+24); out->suffix=body[26]; out->initializing=body[27]!=0;
  out->prefix_cursor=get32(body+28); out->next_logical=get16(body+32); out->exclusion_count=body[34];
  out->candidate={body[36],body[37],body[38]!=0,get32(body+40),get32(body+44),get32(body+48)};
  for (uint8_t i=0; i<MAX_EXCLUSIONS; ++i) out->exclusions[i]=get32(body+EXCLUSIONS_AT+4*i);
  for (uint8_t i=0; i<SLOTS; ++i) {
    const auto* s=body+SLOTS_AT+i*SLOT_STRIDE;
    if (s[15]>1) return false;
    out->slots[i]={get32(s),get32(s+4),get32(s+8),s[12],s[13],s[14],s[15]!=0};
  }
  return valid_image(*out);
}
inline void encode_commit(const Image& image, const uint8_t* body, uint8_t* page) {
  memset(page,0xFF,PAGE_BYTES); put32(page,COMMIT_MAGIC); put32(page+4,image.sequence);
  put32(page+8,get32(body+CRC_AT)); put32(page+12,crc32(page,12));
}
enum class PairKind : uint8_t { free, torn, committed, mismatch };
inline PairKind classify(const uint8_t* body, const uint8_t* commit, Image* out) {
  if (erased(body,BODY_BYTES) && erased(commit,PAGE_BYTES)) return PairKind::free;
  if (get32(commit)!=COMMIT_MAGIC || get32(commit+12)!=crc32(commit,12)) return PairKind::torn;
  if (!valid_body(body,out) || get32(commit+4)!=out->sequence || get32(commit+8)!=get32(body+CRC_AT)) return PairKind::mismatch;
  return PairKind::committed;
}
inline void encode_fence(uint8_t* bytes) {
  memset(bytes,0xFF,2*PAGE_BYTES); put32(bytes,JOURNAL_MAGIC); put16(bytes+4,JOURNAL_VERSION); put16(bytes+6,SLOTS);
  put64(bytes+8,0); put32(bytes+16,0); put32(bytes+20,FENCE_MAGIC);
  put32(bytes+PAGE_BYTES-4,crc32(bytes,PAGE_BYTES-4));
  auto* commit=bytes+PAGE_BYTES; put32(commit,COMMIT_MAGIC); put32(commit+4,0);
  put32(commit+8,get32(bytes+PAGE_BYTES-4)); put32(commit+12,crc32(commit,12));
}
// Merge an intact, uncommitted successor conservatively. No enabling or RF
// binding swap is inferred from a body without a verified commit.
inline void burn(Image& safe, const Image& orphan) {
  if (orphan.next_logical>safe.next_logical) safe.next_logical=orphan.next_logical;
  if (orphan.next_incarnation>safe.next_incarnation) safe.next_incarnation=orphan.next_incarnation;
  if (orphan.prefix_cursor>safe.prefix_cursor) safe.prefix_cursor=orphan.prefix_cursor;
  for (uint8_t i=safe.exclusion_count; i<orphan.exclusion_count; ++i) safe.exclusions[i]=orphan.exclusions[i];
  if (orphan.exclusion_count>safe.exclusion_count) safe.exclusion_count=orphan.exclusion_count;
  for (uint8_t i=0; i<SLOTS; ++i) {
    auto& s=safe.slots[i]; const auto& o=orphan.slots[i];
    if (s.identity && s.identity==o.identity && s.incarnation==o.incarnation && o.next>s.next) s.next=o.next;
    if (s.state && (o.logical_id!=s.logical_id || o.incarnation!=s.incarnation || o.state!=s.state || !o.in_service)) s.in_service=false;
    if (s.state && o.logical_id==s.logical_id && o.state==uint8_t(SlotState::retired)) s.state=o.state;
  }
  auto& c=safe.candidate; const auto& o=orphan.candidate;
  if (c.slot && c.incarnation==o.incarnation && c.identity==o.identity) {
    if (o.next>c.next) c.next=o.next;
    if (o.attempts>c.attempts) c.attempts=o.attempts;
  } else if (c.slot && !o.slot && orphan.slots[c.slot-1].state!=uint8_t(SlotState::paired)) {
    if (!c.replacement) safe.slots[c.slot-1]=SlotRecord{};
    c=Candidate{};
  }
}
struct BankScan {
  bool has_committed=false, has_initial=false, any_nonfree=false, gap=false, broken=false, mismatch=false;
  uint32_t tail=0, vouched_sequence=0;
  Image last, safe;
};
struct Legacy {
  bool has_committed=false, broken=false, unknown=false;
  uint32_t sequence=0;
  uint64_t generation=0;
  uint32_t identity[SLOTS]{}, next[SLOTS]{};
  uint8_t state[SLOTS]{};
};
} // namespace detail

class Journal {
 public:
  explicit Journal(Flash& flash) : flash_(flash) {}

  StorageState open() {
    usable_=false; legacy_=false; torn_tail_=false; image_=detail::Image{};
    active_=0; tail_=0; legacy_image_=detail::Legacy{};
    if (flash_.size()!=REGION_BYTES) return StorageState::corrupt;
    detail::Legacy old[2]; detail::BankScan scan[2];
    for (uint8_t b=0; b<2; ++b) {
      if (!scan_legacy(b,old[b])) return StorageState::corrupt;
      if (!scan_bank(b,scan[b],old[b].has_committed)) return StorageState::corrupt;
    }
    int chosen=-1;
    for (uint8_t b=0; b<2; ++b) if (scan[b].has_committed) {
      if (chosen>=0 && scan[b].last.sequence==scan[chosen].last.sequence) return StorageState::corrupt;
      if (chosen<0 || scan[b].last.sequence>scan[chosen].last.sequence) chosen=b;
    }
    if (chosen>=0) {
      const auto& s=scan[chosen];
      if (s.gap || s.broken || s.mismatch) return StorageState::corrupt;
      active_=uint8_t(chosen); tail_=s.tail; image_=s.safe;
      const auto& other=scan[active_^1];
      // A verified newer commit with an invalid/unknown body must not fall
      // back to an older bank and thereby replay counters or identity claims.
      if (other.vouched_sequence>image_.sequence) return StorageState::corrupt;
      if (other.has_initial && other.safe.sequence>image_.sequence) {
        if (other.safe.sequence!=image_.sequence+1 || !detail::valid_transition(image_,other.safe)) return StorageState::corrupt;
        detail::burn(image_,other.safe); torn_tail_=true;
      }
      if (!image_.initializing && (old[0].has_committed || old[1].has_committed || old[0].unknown || old[1].unknown))
        return StorageState::corrupt;
      if (!image_.initializing && !fence_valid(active_)) return StorageState::corrupt;
      torn_tail_=torn_tail_ || (s.any_nonfree && s.tail && !last_committed(active_,s.tail-1));
      usable_=true; return state();
    }
    // An intact first initialization body may have lost its commit. It cannot
    // have emitted RF: only finishing the durable initialization is permitted.
    for (uint8_t b=0; b<2; ++b) if (scan[b].has_initial && scan[b].safe.initializing && scan[b].safe.sequence==1) {
      if (scan[b].broken || scan[b].mismatch || scan[b].gap) return StorageState::corrupt;
      image_=scan[b].safe; active_=b; tail_=scan[b].tail;
      usable_=true; torn_tail_=true; return state();
    }
    for (uint8_t b=0; b<2; ++b) if (old[b].has_committed) {
      if (chosen>=0 && old[b].sequence==old[chosen].sequence) return StorageState::corrupt;
      if (chosen<0 || old[b].sequence>old[chosen].sequence) chosen=b;
    }
    if (chosen>=0) {
      if (old[chosen].broken || old[chosen].unknown || old[chosen^1].unknown) return StorageState::corrupt;
      legacy_image_=old[chosen]; active_=uint8_t(chosen);
      usable_=true; legacy_=true; return state();
    }
    if (old[0].unknown || old[1].unknown || scan[0].mismatch || scan[1].mismatch ||
        scan[0].broken || scan[1].broken || scan[0].gap || scan[1].gap ||
        scan[0].has_initial || scan[1].has_initial) return StorageState::corrupt;
    // Only erased storage or a recognizable interrupted v2 initialization.
    if (scan[1].any_nonfree) return StorageState::corrupt;
    // An owned magic alone cannot authorize formatting an invalid image.
    // Only a prefix of the first empty-storage initializer can be resumed.
    for (uint32_t pair=0; pair<scan[0].tail; ++pair) {
      if (!read_pair(0,pair) || !initialization_fragment(body_,commit_)) return StorageState::corrupt;
    }
    tail_=scan[0].tail; torn_tail_=scan[0].any_nonfree;
    usable_=true; return state();
  }
  StorageState state() const {
    if (!usable_) return StorageState::corrupt;
    if (legacy_) return StorageState::legacy;
    if (!image_.generation) return StorageState::empty;
    if (image_.initializing) return StorageState::initializing;
    return free_pairs() ? StorageState::ready : StorageState::full;
  }
  bool torn_tail_skipped() const { return torn_tail_; }
  // Boolean policy check only: no private RF identity, seed or counter export.
  bool enrollment_profile_matches(uint8_t suffix) const {
    return require_ready()==Status::ok && image_.suffix==suffix;
  }
  bool generation_hex(char out[17]) const {
    if (!usable_ || legacy_ || !image_.generation) return false;
    const char* digits="0123456789ABCDEF";
    for (int i=0; i<16; ++i) out[i]=digits[(image_.generation>>(60-4*i))&15];
    out[16]=0; return true;
  }
  Shutter shutter(uint8_t id) const {
    Shutter result; const auto* s=slot(id);
    if (!s || !s->state || s->state==uint8_t(SlotState::retired)) return result;
    result.shutter_id=id; result.state=SlotState(s->state); result.last_command=s->command;
    result.logical_id=s->logical_id; result.in_service=s->in_service; result.incarnation=s->incarnation;
    if (image_.candidate.slot==id) {
      result.has_candidate=true; result.replacement=image_.candidate.replacement;
      result.attempts=image_.candidate.attempts; result.candidate_incarnation=image_.candidate.incarnation;
    }
    return result;
  }
  uint32_t incarnation(uint8_t id, bool candidate=false) const {
    const auto* s=slot(id);
    if (!s || s->state==uint8_t(SlotState::retired)) return 0;
    return candidate ? image_.candidate.slot==id ? image_.candidate.incarnation : 0 : s->incarnation;
  }
  bool identity(uint8_t id, uint32_t* out, bool candidate=false) const {
    const auto* s=slot(id);
    if (!out || !s || !s->state || s->state==uint8_t(SlotState::retired)) return false;
    const uint32_t value=candidate ? image_.candidate.slot==id ? image_.candidate.identity : 0 : s->identity;
    if (!value) return false;
    *out=value; return true;
  }
  bool next_counter(uint8_t id, uint32_t* out, bool candidate=false) const {
    const auto* s=slot(id);
    if (!out || !s || !s->state || s->state==uint8_t(SlotState::retired) ||
        (candidate ? image_.candidate.slot!=id : !s->identity)) return false;
    *out=candidate ? image_.candidate.next : s->next; return true;
  }
  uint8_t find_identity(uint32_t id) const {
    if (!usable_ || legacy_ || !id) return 0;
    if (image_.candidate.identity==id) return image_.candidate.slot;
    for (uint8_t i=0; i<SLOTS; ++i) if (image_.slots[i].identity==id) return i+1;
    return 0;
  }

  Status initialize(uint64_t generation, uint16_t seed, uint8_t suffix=1) {
    if (!usable_) return Status::corrupt;
    if (!generation) return Status::bad_argument;
    if (state()!=StorageState::empty && !legacy_) return Status::bad_argument;
    detail::Image next; next.generation=generation; next.sequence=1;
    next.identity_seed=seed; next.suffix=suffix; next.initializing=true;
    if (legacy_) {
      for (uint8_t i=0; i<SLOTS; ++i) if (legacy_image_.identity[i])
        next.exclusions[next.exclusion_count++]=legacy_image_.identity[i];
      const uint8_t target=active_^1;
      // The selected valid v1 bank stays intact until the v2 reset marker is
      // verified. Losing power while preparing its other bank loses no keys.
      for (uint32_t sector=0; sector<SECTORS_PER_BANK; ++sector) {
        bool blank=false;
        if (!sector_erased(target,sector,blank)) return fail();
        if (!blank && (!flash_.erase_sector(target*BANK_BYTES+sector*SECTOR_BYTES) ||
            !sector_erased(target,sector,blank) || !blank)) return fail();
      }
      const Status result=write_pair(target,0,next);
      if (result!=Status::ok) return result;
      active_=target; tail_=1;
    } else {
      if (tail_>=PAIRS_PER_BANK) return Status::no_space;
      const Status result=write_pair(active_,tail_,next);
      if (result!=Status::ok) return result;
      ++tail_;
    }
    image_=next; legacy_=false; return Status::ok;
  }
  Status allocate_candidate(uint8_t id, bool replacement) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    if (!slot(id)) return Status::unknown_slot;
    if (image_.candidate.slot) return Status::busy;
    const auto& current=image_.slots[id-1];
    if (replacement) {
      if (current.state!=uint8_t(SlotState::paired)) return Status::not_paired;
      if (current.in_service) return Status::disabled;
    } else if (current.state && current.state!=uint8_t(SlotState::retired)) return Status::inventory_full;
    detail::Image next=image_;
    if (!replacement) {
      if (next.next_logical>254) return Status::logical_exhausted;
      next.slots[id-1]=detail::SlotRecord{};
      next.slots[id-1].logical_id=uint8_t(next.next_logical++);
      next.slots[id-1].state=uint8_t(SlotState::pending);
    }
    if (next.next_incarnation==UINT32_MAX) return Status::identity_exhausted;
    uint32_t identity=0;
    while (next.prefix_cursor<RF_PREFIXES) {
      const uint16_t prefix=uint16_t(next.identity_seed+next.prefix_cursor++);
      identity=(uint32_t(prefix)<<8)|next.suffix;
      if (prefix && !detail::excluded(next,identity)) break;
      identity=0;
    }
    if (!identity) return Status::identity_exhausted;
    next.candidate={id,0,replacement,identity,0,next.next_incarnation++};
    next.slots[id-1].in_service=false;
    return append(next,false);
  }
  Status claim_attempt(uint8_t id, bool retry) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    if (!slot(id)) return Status::unknown_slot;
    if (image_.candidate.slot!=id) return Status::no_candidate;
    const auto& c=image_.candidate;
    if (retry ? c.attempts!=1 || c.next!=2 : c.attempts!=0 || c.next!=0) return Status::retry_unavailable;
    detail::Image next=image_; next.candidate.attempts=retry ? 2 : 1;
    return append(next,false);
  }
  Status confirm_candidate(uint8_t id) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    if (!slot(id)) return Status::unknown_slot;
    if (image_.candidate.slot!=id) return Status::no_candidate;
    const auto& c=image_.candidate;
    if (!c.attempts || c.next<2) return Status::no_attempt;
    detail::Image next=image_; auto& s=next.slots[id-1];
    s.identity=c.identity; s.next=c.next; s.incarnation=c.incarnation;
    s.state=uint8_t(SlotState::paired); s.command=0; s.in_service=true;
    next.candidate=detail::Candidate{}; return append(next,false);
  }
  Status cancel_candidate(uint8_t id) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    if (!slot(id)) return Status::unknown_slot;
    if (image_.candidate.slot!=id) return Status::no_candidate;
    detail::Image next=image_;
    if (!next.candidate.replacement) next.slots[id-1]=detail::SlotRecord{};
    else next.slots[id-1].in_service=false;
    next.candidate=detail::Candidate{}; return append(next,false);
  }
  Status set_service(uint8_t id, bool enabled) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    const auto* s=slot(id);
    if (!s) return Status::unknown_slot;
    if (s->state!=uint8_t(SlotState::paired)) return Status::not_paired;
    if (image_.candidate.slot==id) return Status::busy;
    if (s->in_service==enabled) return Status::ok;
    detail::Image next=image_; next.slots[id-1].in_service=enabled;
    return append(next,false);
  }
  Status retire(uint8_t id) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    const auto* s=slot(id);
    if (!s) return Status::unknown_slot;
    if (s->state==uint8_t(SlotState::retired)) return Status::ok;
    if (s->state!=uint8_t(SlotState::paired)) return Status::not_paired;
    if (s->in_service) return Status::disabled;
    if (image_.candidate.slot==id) return Status::busy;
    detail::Image next=image_; next.slots[id-1].state=uint8_t(SlotState::retired);
    return append(next,false);
  }

  // Trusted primitive entry point for codec/backend consumers. Arbitrary RF
  // keys are retained in a bounded exclusion set, including after retirement.
  // It never initializes legacy/corrupt storage or bypasses a live candidate.
  Status provision(uint8_t id, const NewController& fresh, Shutter* out=nullptr, bool* created=nullptr) {
    if (!usable_) return Status::corrupt;
    if (created) *created=false;
    if (id<1 || id>SLOTS) return Status::unknown_slot;
    if (state()==StorageState::empty) {
      if (!fresh.generation || !fresh.identity || fresh.identity>0xFFFFFF) return Status::bad_argument;
      Status result=initialize(fresh.generation,0,1);
      if (result!=Status::ok) return result;
      while (state()==StorageState::initializing) if ((result=maintain())!=Status::ok) return result;
    }
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    const auto& s=image_.slots[id-1];
    if (s.state && s.state!=uint8_t(SlotState::retired)) {
      if (out) *out=shutter(id);
      return Status::ok;
    }
    if (image_.candidate.slot) return Status::busy;
    if (!fresh.identity || fresh.identity>0xFFFFFF) return Status::bad_argument;
    if (detail::known_identity(image_,fresh.identity)) return Status::identity_in_use;
    if (image_.exclusion_count==MAX_EXCLUSIONS) return Status::identity_exhausted;
    if (image_.next_logical>254) return Status::logical_exhausted;
    if (image_.next_incarnation==UINT32_MAX) return Status::identity_exhausted;
    detail::Image next=image_; next.exclusions[next.exclusion_count++]=fresh.identity;
    next.slots[id-1]={fresh.identity,fresh.first_counter,next.next_incarnation++,uint8_t(next.next_logical++),uint8_t(SlotState::pending),0,false};
    const Status result=append(next,false);
    if (result==Status::ok) { if (created) *created=true; if (out) *out=shutter(id); }
    return result;
  }
  Status confirm(uint8_t id, Shutter* out=nullptr) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    const auto* s=slot(id);
    if (!s || !s->identity || s->state==uint8_t(SlotState::retired)) return Status::unknown_slot;
    if (image_.candidate.slot==id) return Status::busy;
    if (s->state!=uint8_t(SlotState::paired)) {
      detail::Image next=image_; next.slots[id-1].state=uint8_t(SlotState::paired); next.slots[id-1].in_service=true;
      const Status result=append(next,false); if (result!=Status::ok) return result;
    }
    if (out) *out=shutter(id);
    return Status::ok;
  }
  Status reserve(uint8_t id, uint8_t command, bool critical, Reservation* out,
                 uint32_t epoch=0, bool candidate=false) {
    const Status ready=require_ready(); if (ready!=Status::ok) return ready;
    const auto* s=slot(id);
    if (!s || !s->state || s->state==uint8_t(SlotState::retired)) return Status::unknown_slot;
    if (!out) return Status::bad_argument;
    uint32_t identity=0, next_counter=0, actual_epoch=0;
    if (candidate) {
      if (image_.candidate.slot!=id) return Status::no_candidate;
      const auto& c=image_.candidate;
      if (!c.attempts || c.next>=2u*c.attempts) return Status::no_attempt;
      identity=c.identity; next_counter=c.next; actual_epoch=c.incarnation;
    } else {
      if (!s->identity) return Status::unknown_slot;
      identity=s->identity; next_counter=s->next; actual_epoch=s->incarnation;
      if (epoch && (s->state!=uint8_t(SlotState::paired) || (!critical && (!s->in_service || image_.candidate.slot==id))))
        return s->state!=uint8_t(SlotState::paired) ? Status::not_paired : Status::disabled;
    }
    if (epoch && epoch!=actual_epoch) return Status::incarnation_mismatch;
    if (next_counter>=MAX_NEXT || (!critical && next_counter>=MAX_NEXT-1)) return Status::exhausted;
    detail::Image next=image_;
    if (candidate) next.candidate.next=next_counter+1;
    else { next.slots[id-1].next=next_counter+1; next.slots[id-1].command=command; }
    const Status result=append(next,critical);
    if (result!=Status::ok) return result;
    *out={id,identity,uint16_t(next_counter),actual_epoch}; return Status::ok;
  }

  bool maintenance_due() const {
    return usable_ && !legacy_ && image_.generation && (image_.initializing || free_pairs()<=MAINTAIN_BELOW);
  }
  Status maintain() {
    if (!usable_) return Status::corrupt;
    if (!maintenance_due()) return Status::ok;
    if (image_.sequence==UINT32_MAX) return Status::no_space;
    if (image_.initializing) {
      if (!fence_valid(active_)) {
        bool blank=false;
        if (!sector_erased(active_,SECTORS_PER_BANK-1,blank)) return fail();
        // Initializing images have no registered shutters or RF reservations;
        // only their first snapshots precede this otherwise unused sector.
        if (tail_*RECORD_BYTES> BANK_BYTES-SECTOR_BYTES) return Status::corrupt;
        if (!blank) {
          const uint32_t off=active_*BANK_BYTES+BANK_BYTES-SECTOR_BYTES;
          if (!flash_.erase_sector(off) || !sector_erased(active_,SECTORS_PER_BANK-1,blank) || !blank) return fail();
          return Status::ok;
        }
        return write_fence(active_);
      }
      const uint8_t other=active_^1;
      for (uint32_t sector=0; sector<SECTORS_PER_BANK; ++sector) {
        bool blank=false;
        if (!sector_erased(other,sector,blank)) return fail();
        if (!blank) {
          const uint32_t off=other*BANK_BYTES+sector*SECTOR_BYTES;
          if (!flash_.erase_sector(off) || !sector_erased(other,sector,blank) || !blank) return fail();
          return Status::ok;
        }
      }
      detail::Image next=image_; next.initializing=false;
      return append(next,false);
    }
    const uint8_t other=active_^1;
    const bool fenced=fence_valid(other);
    for (uint32_t sector=0; sector<SECTORS_PER_BANK; ++sector) {
      bool blank=false;
      if (fenced && sector==SECTORS_PER_BANK-1) {
        if (!range_erased(other*BANK_BYTES+sector*SECTOR_BYTES,FENCE_AT-sector*SECTOR_BYTES,blank)) return fail();
      } else if (!sector_erased(other,sector,blank)) return fail();
      if (!blank) {
        const uint32_t off=other*BANK_BYTES+sector*SECTOR_BYTES;
        if (!flash_.erase_sector(off) || !sector_erased(other,sector,blank) || !blank) return fail();
        return Status::ok;
      }
    }
    if (!fenced) return write_fence(other);
    detail::Image next=image_; next.sequence=image_.sequence+1;
    const Status result=write_pair(other,0,next);
    if (result!=Status::ok) return result;
    active_=other; tail_=1; image_=next; return Status::ok;
  }
 private:
  uint32_t free_pairs() const { return tail_<PAIRS_PER_BANK ? PAIRS_PER_BANK-tail_ : 0; }
  Status require_ready() const {
    if (!usable_) return Status::corrupt;
    return legacy_ || !image_.generation || image_.initializing ? Status::not_initialized : Status::ok;
  }
  const detail::SlotRecord* slot(uint8_t id) const {
    return usable_ && !legacy_ && id>=1 && id<=SLOTS ? &image_.slots[id-1] : nullptr;
  }
  Status fail() { usable_=false; return Status::io_error; }
  static uint32_t pair_offset(uint8_t bank, uint32_t pair) { return bank*BANK_BYTES+pair*RECORD_BYTES; }
  bool read_pair(uint8_t bank, uint32_t pair) {
    const uint32_t offset=pair_offset(bank,pair);
    return flash_.read(offset,body_,BODY_BYTES) && flash_.read(offset+BODY_BYTES,commit_,PAGE_BYTES);
  }
  bool program_verified(uint32_t offset, const uint8_t* bytes) {
    return flash_.program_page(offset,bytes) && flash_.read(offset,check_,PAGE_BYTES) && !memcmp(bytes,check_,PAGE_BYTES);
  }
  Status write_pair(uint8_t bank, uint32_t pair, const detail::Image& image) {
    if (!detail::valid_image(image)) return Status::bad_argument;
    detail::encode_body(image,body_); detail::encode_commit(image,body_,commit_);
    const uint32_t offset=pair_offset(bank,pair);
    for (uint32_t page=0; page<BODY_BYTES; page+=PAGE_BYTES)
      if (!program_verified(offset+page,body_+page)) return fail();
    if (!program_verified(offset+BODY_BYTES,commit_)) return fail();
    return Status::ok;
  }
  Status append(detail::Image next, bool critical) {
    if (!usable_ || legacy_) return Status::corrupt;
    if (image_.sequence==UINT32_MAX || free_pairs()<=(critical ? 0u : CRITICAL_PAIRS)) return Status::no_space;
    next.sequence=image_.sequence+1;
    if (!detail::valid_image(next) || !detail::valid_transition(image_,next)) return Status::bad_argument;
    const Status result=write_pair(active_,tail_,next);
    if (result!=Status::ok) return result;
    ++tail_; image_=next; return Status::ok;
  }
  bool range_erased(uint32_t offset, uint32_t length, bool& blank) {
    blank=true;
    while (length) {
      const uint32_t size=length<PAGE_BYTES ? length : PAGE_BYTES;
      if (!flash_.read(offset,check_,size)) return false;
      if (!detail::erased(check_,size)) blank=false;
      offset+=size; length-=size;
    }
    return true;
  }
  bool sector_erased(uint8_t bank, uint32_t sector, bool& blank) {
    return range_erased(bank*BANK_BYTES+sector*SECTOR_BYTES,SECTOR_BYTES,blank);
  }
  bool fence_valid(uint8_t bank) {
    detail::encode_fence(body_);
    for (uint32_t page=0; page<2*PAGE_BYTES; page+=PAGE_BYTES) {
      if (!flash_.read(bank*BANK_BYTES+FENCE_AT+page,check_,PAGE_BYTES) || memcmp(check_,body_+page,PAGE_BYTES)) return false;
    }
    return true;
  }
  Status write_fence(uint8_t bank) {
    detail::encode_fence(body_);
    for (uint32_t page=0; page<2*PAGE_BYTES; page+=PAGE_BYTES)
      if (!program_verified(bank*BANK_BYTES+FENCE_AT+page,body_+page)) return fail();
    return Status::ok;
  }
  bool last_committed(uint8_t bank, uint32_t pair) {
    detail::Image value;
    return read_pair(bank,pair) && detail::classify(body_,commit_,&value)==detail::PairKind::committed;
  }
  // A partially programmed magic/version can occur during an explicit first
  // initialization. It is recognizable only at the beginning of a record.
  static bool v2_fragment(const uint8_t* body) {
    uint8_t prefix[8]; detail::put32(prefix,detail::JOURNAL_MAGIC);
    detail::put16(prefix+4,detail::JOURNAL_VERSION); detail::put16(prefix+6,SLOTS);
    bool written=false, partial=false;
    for (size_t i=0; i<sizeof(prefix); ++i) {
      if (body[i]==prefix[i] && !partial) { written=true; continue; }
      if ((body[i]&prefix[i])!=prefix[i]) return false;
      partial=true;
      if (body[i]!=0xFF) written=true;
    }
    return written;
  }
  static bool initialization_fragment(const uint8_t* body, const uint8_t* commit) {
    if (!v2_fragment(body) || !detail::erased(commit,PAGE_BYTES)) return false;
    detail::Image initial;
    initial.generation=detail::get64(body+8);
    if (!initial.generation) return false;
    initial.sequence=1; initial.initializing=true;
    initial.identity_seed=detail::get16(body+24); initial.suffix=body[26];
    uint8_t expected[BODY_BYTES]; detail::encode_body(initial,expected);
    bool incomplete=false;
    for (uint32_t page=0; page<BODY_BYTES; page+=PAGE_BYTES) {
      if (incomplete) return detail::erased(body+page,BODY_BYTES-page);
      for (uint32_t i=page; i<page+PAGE_BYTES; ++i) {
        if (body[i]==expected[i]) continue;
        if ((body[i]&expected[i])!=expected[i] ||
            !detail::erased(body+i+1,page+PAGE_BYTES-i-1)) return false;
        incomplete=true; break;
      }
    }
    return incomplete;
  }
  bool scan_bank(uint8_t bank, detail::BankScan& scan, bool legacy_bank) {
    if (legacy_bank) return true;
    bool seen_free=false;
    for (uint32_t pair=0; pair<PAIRS_PER_BANK; ++pair) {
      if (!read_pair(bank,pair)) return false;
      detail::Image value;
      const auto kind=detail::classify(body_,commit_,&value);
      if (kind==detail::PairKind::free) { seen_free=true; continue; }
      scan.any_nonfree=true; scan.tail=pair+1;
      if (seen_free) scan.gap=true;
      if (kind==detail::PairKind::committed || kind==detail::PairKind::mismatch) {
        const uint32_t sequence=detail::get32(commit_+4);
        if (sequence>scan.vouched_sequence) scan.vouched_sequence=sequence;
      }
      if (kind==detail::PairKind::mismatch) { scan.mismatch=true; continue; }
      const bool valid=detail::valid_body(body_,&value);
      if (!valid) {
        if (!v2_fragment(body_)) scan.broken=true;
        continue;
      }
      if (kind==detail::PairKind::torn) {
        if (scan.has_committed) {
          if (value.sequence!=scan.last.sequence+1 || !detail::valid_transition(scan.safe,value)) scan.broken=true;
          else detail::burn(scan.safe,value);
        } else {
          if (scan.has_initial && (!value.initializing || value.sequence!=1) &&
              (value.sequence!=scan.safe.sequence || !detail::valid_transition(scan.safe,value))) scan.broken=true;
          scan.has_initial=true; scan.safe=value;
        }
        continue;
      }
      if (scan.has_committed) {
        if (value.sequence!=scan.last.sequence+1 || !detail::valid_transition(scan.safe,value)) scan.broken=true;
      } else if (scan.has_initial && !(value.initializing && value.sequence==1)) {
        if (value.sequence!=scan.safe.sequence+1 || !detail::valid_transition(scan.safe,value)) scan.broken=true;
      }
      scan.last=value; scan.safe=value; scan.has_committed=true;
    }
    return true;
  }
  bool scan_legacy(uint8_t bank, detail::Legacy& scan) {
    bool seen_free=false;
    for (uint32_t pair=0; pair<BANK_BYTES/(2*PAGE_BYTES); ++pair) {
      const uint32_t offset=bank*BANK_BYTES+pair*2*PAGE_BYTES;
      if (!flash_.read(offset,body_,PAGE_BYTES) || !flash_.read(offset+PAGE_BYTES,commit_,PAGE_BYTES)) return false;
      if (detail::erased(body_,PAGE_BYTES) && detail::erased(commit_,PAGE_BYTES)) { seen_free=true; continue; }
      if (detail::get32(body_)!=detail::JOURNAL_MAGIC) continue;
      const uint16_t version=detail::get16(body_+4);
      if (version==2 || version==0xFFFF) continue;
      if (version!=1) {
        // Programming may stop inside the version field. A short owned
        // header fragment with an erased remainder/commit is not a future
        // committed format, and no RF could have followed this write.
        if (v2_fragment(body_) && detail::erased(body_+8,PAGE_BYTES-8) && detail::erased(commit_,PAGE_BYTES)) continue;
        scan.unknown=true; continue;
      }
      const bool body_valid=detail::get16(body_+6)==SLOTS && detail::get64(body_+8) && detail::get32(body_+16) &&
        detail::get32(body_+PAGE_BYTES-4)==detail::crc32(body_,PAGE_BYTES-4);
      const bool committed=detail::get32(commit_)==detail::COMMIT_MAGIC && detail::get32(commit_+12)==detail::crc32(commit_,12);
      if (!body_valid) { if (committed) scan.broken=true; continue; }
      const uint32_t seq=detail::get32(body_+16);
      const uint64_t generation=detail::get64(body_+8);
      uint32_t identities[SLOTS]{}, counters[SLOTS]{}; uint8_t states[SLOTS]{};
      bool semantic=true;
      for (uint8_t i=0; i<SLOTS; ++i) {
        const auto* s=body_+24+12*i; identities[i]=detail::get32(s); counters[i]=detail::get32(s+4); states[i]=s[8];
        if (states[i]>2 || (!states[i] && (identities[i] || counters[i] || s[9])) ||
            (states[i] && (!identities[i] || identities[i]>0xFFFFFF || counters[i]>MAX_NEXT))) semantic=false;
        for (uint8_t j=0; j<i; ++j) if (states[i] && identities[i]==identities[j]) semantic=false;
        if (scan.has_committed && scan.state[i] && (identities[i]!=scan.identity[i] || counters[i]<scan.next[i] || states[i]<scan.state[i])) semantic=false;
      }
      if (scan.has_committed && (generation!=scan.generation || seq!=scan.sequence+1)) semantic=false;
      if (!semantic) { scan.broken=true; continue; }
      if (!committed) {
        // Include provisioned keys from an intact uncommitted successor too.
        if (scan.has_committed) for (uint8_t i=0; i<SLOTS; ++i) if (identities[i]) scan.identity[i]=identities[i];
        continue;
      }
      if (detail::get32(commit_+4)!=seq || detail::get32(commit_+8)!=detail::get32(body_+PAGE_BYTES-4)) { scan.broken=true; continue; }
      if (seen_free) scan.broken=true;
      scan.sequence=seq; scan.generation=generation; scan.has_committed=true;
      for (uint8_t i=0; i<SLOTS; ++i) { scan.identity[i]=identities[i]; scan.next[i]=counters[i]; scan.state[i]=states[i]; }
    }
    return true;
  }

  Flash& flash_;
  detail::Image image_;
  detail::Legacy legacy_image_;
  uint8_t active_=0;
  uint32_t tail_=0;
  bool usable_=false, legacy_=false, torn_tail_=false;
  uint8_t body_[BODY_BYTES]{}, commit_[PAGE_BYTES]{}, check_[PAGE_BYTES]{};
};

} // namespace journal
} // namespace x2d
