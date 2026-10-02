#pragma once

// Durable identity/counter journal for up to 16 shutters (shutter_id 1..16).
// Independent of Home Assistant, USB, the radio and protocol.h. The firmware
// is the only owner of rolling counters; the journal never resets or wraps one.
//
// Flash region: 64 KiB, relative offsets 0..65535, two banks of 32 KiB. Each
// bank is append-only: a record is one 256-byte body page (version, journal
// generation, sequence, all 16 slots, CRC-32) followed by a 256-byte commit
// page (sequence, body CRC, its own CRC) programmed only after the body read
// back correctly. The newest committed sequence is the state. When the active
// bank runs low, maintain() erases the other bank one sector per call and then
// writes a snapshot there; erases never happen inside reserve().
//
// Torn tails (interrupted program) are skipped, never rewritten. Fully erased
// storage is a valid empty journal. Anything that vouches for a record it
// cannot back (commit without matching body, unknown version, sequence gap,
// slot regression, data after free space, equal sequences in both banks) is
// corruption: every mutation then fails and nothing is formatted.
//
// Counters never roll back. A pair after the newest commit whose body is
// intact but whose commit page is erased, torn or damaged is treated as burnt:
// open() advances every provisioned slot's next counter to that body's value
// (an interrupted write wastes one counter; a damaged commit cannot revive an
// emitted one). Such a body that is not the direct successor of the newest
// commit is corruption. Identity, pending/paired and last_command come from
// the newest commit only, so an unacknowledged or damaged confirm may need to
// be repeated. Not covered: simultaneous damage to both pages of a record.
//
// STOP guarantee and its ceiling. Reserved for critical (STOP) reservations:
// the last CRITICAL_PAIRS = 16 records of the active bank (one STOP per slot,
// enough for a full queue) and counter 0xFFFF of every identity. Movements
// and provision/confirm stop earlier, so those STOPs never need an erase.
// This is a ceiling, not an unbounded STOP budget: the scheduler must run
// maintain() at idle whenever maintenance_due() is true, before it admits more
// movements, and must coalesce pending STOPs per slot (one reservation covers
// every copy of one STOP).

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ha_x2d {
namespace journal {

constexpr uint8_t SLOTS = 16;  // keep equal to ha_x2d::MAX_SHUTTERS
constexpr uint32_t PAGE_BYTES = 256;
constexpr uint32_t SECTOR_BYTES = 4096;
constexpr uint32_t BANK_BYTES = 32768;
constexpr uint32_t REGION_BYTES = 2 * BANK_BYTES;
constexpr uint32_t PAIRS_PER_BANK = BANK_BYTES / (2 * PAGE_BYTES);
constexpr uint32_t SECTORS_PER_BANK = BANK_BYTES / SECTOR_BYTES;
constexpr uint32_t CRITICAL_PAIRS = SLOTS;  // last records are reserved for STOP
// maintain() acts at or below this many free records, which leaves 16
// movement records to finish rotating before movements are refused.
constexpr uint32_t MAINTAIN_BELOW = CRITICAL_PAIRS + 16;
constexpr uint32_t MAX_NEXT = 0x10000;      // next counter; above 0xFFFF = exhausted
static_assert(MAINTAIN_BELOW < PAIRS_PER_BANK, "bank too small for the STOP reserve");

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

enum class StorageState : uint8_t { empty, ready, corrupt, full };
enum class SlotState : uint8_t { unused = 0, pending = 1, paired = 2 };
enum class Status : uint8_t {
  ok,
  unknown_slot,     // shutter_id outside 1..16 or never provisioned
  bad_argument,     // invalid identity/generation
  identity_in_use,  // draw another identity and retry
  exhausted,        // counter 0xFFFF was already reserved; no wrap
  no_space,         // run maintain() at idle (STOP may still pass)
  corrupt,          // not opened, corrupt, or failed since open()
  io_error,         // this write failed; open() again to recover
};

inline const char* state_name(StorageState state) {
  switch (state) {
    case StorageState::empty: return "empty";
    case StorageState::ready: return "ready";
    case StorageState::full: return "full";
    default: return "corrupt";
  }
}
inline const char* slot_state_name(SlotState state) {
  return state == SlotState::pending ? "pending"
       : state == SlotState::paired  ? "paired" : nullptr;
}
// Protocol error string for a failed call, nullptr for ok.
inline const char* protocol_error(Status status) {
  switch (status) {
    case Status::ok: return nullptr;
    case Status::unknown_slot: return "unknown_shutter";
    case Status::exhausted: return "counter_exhausted";
    case Status::no_space: return "storage_full";
    case Status::bad_argument:
    case Status::identity_in_use: return "invalid_request";
    default: return "storage_corrupt";
  }
}

// Public view of a slot: no identity, no counter.
struct Shutter {
  uint8_t shutter_id = 0;
  SlotState state = SlotState::unused;
  uint8_t last_command = 0;  // opaque tag of the last reservation; 0 = none
};

// Explicit inputs for a new controller; the journal invents nothing.
// generation is used only to initialize empty storage (nonzero, random).
struct NewController {
  uint32_t identity = 0;       // raw RF identity, 1..0xFFFFFF
  uint16_t first_counter = 0;  // first clear counter handed out
  uint64_t generation = 0;
};

// PRIVATE: contains the raw RF identity. Never export it in diagnostics.
struct Reservation {
  uint8_t shutter_id = 0;
  uint32_t identity = 0;
  uint16_t counter = 0;
};

namespace detail {

constexpr uint32_t JOURNAL_MAGIC = 0x4A443258;         // "X2DJ"
constexpr uint32_t COMMIT_MAGIC = 0x43443258;  // "X2DC"
constexpr uint16_t JOURNAL_VERSION = 1;
constexpr size_t SLOT_STRIDE = 12;
constexpr size_t SLOTS_AT = 24;
constexpr size_t CRC_AT = PAGE_BYTES - 4;
static_assert(SLOTS_AT + SLOTS * SLOT_STRIDE <= CRC_AT, "record does not fit");

inline uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  while (length--) {
    crc ^= *data++;
    for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
inline void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline void put64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline uint16_t get16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get32(const uint8_t* p) {
  return uint32_t{p[0]} | (uint32_t{p[1]} << 8) | (uint32_t{p[2]} << 16) | (uint32_t{p[3]} << 24);
}
inline uint64_t get64(const uint8_t* p) { return get32(p) | (uint64_t{get32(p + 4)} << 32); }
inline bool erased(const uint8_t* p, size_t length) {
  while (length--) if (*p++ != 0xFF) return false;
  return true;
}

struct SlotRecord {
  uint32_t identity = 0;
  uint32_t next = 0;  // next counter to hand out, 0..MAX_NEXT
  uint8_t state = 0;
  uint8_t command = 0;
};
struct Image {
  uint64_t generation = 0;
  uint32_t sequence = 0;
  SlotRecord slots[SLOTS];
};

inline bool valid_image(const Image& image) {
  if (!image.generation || !image.sequence) return false;
  for (uint8_t i = 0; i < SLOTS; ++i) {
    const SlotRecord& s = image.slots[i];
    if (s.state > static_cast<uint8_t>(SlotState::paired)) return false;
    if (s.state == 0) {
      if (s.identity || s.next || s.command) return false;
      continue;
    }
    if (!s.identity || s.identity > 0xFFFFFF || s.next > MAX_NEXT) return false;
    for (uint8_t j = 0; j < i; ++j)
      if (image.slots[j].state && image.slots[j].identity == s.identity) return false;
  }
  return true;
}

// Identity, state and counter never move backwards inside one journal.
inline bool valid_transition(const Image& before, const Image& after) {
  if (before.generation != after.generation) return false;
  for (uint8_t i = 0; i < SLOTS; ++i) {
    const SlotRecord& p = before.slots[i];
    const SlotRecord& n = after.slots[i];
    if (p.state && (n.state < p.state || n.identity != p.identity || n.next < p.next))
      return false;
  }
  return true;
}

inline void encode_body(const Image& image, uint8_t* page) {
  memset(page, 0xFF, PAGE_BYTES);
  put32(page, JOURNAL_MAGIC);
  put16(page + 4, JOURNAL_VERSION);
  put16(page + 6, SLOTS);
  put64(page + 8, image.generation);
  put32(page + 16, image.sequence);
  put32(page + 20, 0);
  for (uint8_t i = 0; i < SLOTS; ++i) {
    uint8_t* s = page + SLOTS_AT + i * SLOT_STRIDE;
    put32(s, image.slots[i].identity);
    put32(s + 4, image.slots[i].next);
    s[8] = image.slots[i].state;
    s[9] = image.slots[i].command;
    s[10] = s[11] = 0;
  }
  put32(page + CRC_AT, crc32(page, CRC_AT));
}

inline void encode_commit(const Image& image, const uint8_t* body, uint8_t* page) {
  memset(page, 0xFF, PAGE_BYTES);
  put32(page, COMMIT_MAGIC);
  put32(page + 4, image.sequence);
  put32(page + 8, get32(body + CRC_AT));
  put32(page + 12, crc32(page, 12));
}

enum class PairKind : uint8_t { free, torn, committed, mismatch };

// Structurally and semantically valid body page (CRC, magic, version, slots).
inline bool valid_body(const uint8_t* body, Image* out) {
  if (get32(body + CRC_AT) != crc32(body, CRC_AT) || get32(body) != JOURNAL_MAGIC ||
      get16(body + 4) != JOURNAL_VERSION || get16(body + 6) != SLOTS)
    return false;
  out->generation = get64(body + 8);
  out->sequence = get32(body + 16);
  for (uint8_t i = 0; i < SLOTS; ++i) {
    const uint8_t* s = body + SLOTS_AT + i * SLOT_STRIDE;
    out->slots[i].identity = get32(s);
    out->slots[i].next = get32(s + 4);
    out->slots[i].state = s[8];
    out->slots[i].command = s[9];
  }
  return valid_image(*out);
}

// free: both pages erased. torn: not a complete commit (interrupted write).
// mismatch: a valid commit page vouches for a body that is missing, damaged,
// of another version, or semantically invalid.
inline PairKind classify(const uint8_t* body, const uint8_t* commit, Image* out) {
  if (erased(body, PAGE_BYTES) && erased(commit, PAGE_BYTES)) return PairKind::free;
  if (get32(commit) != COMMIT_MAGIC || get32(commit + 12) != crc32(commit, 12))
    return PairKind::torn;
  if (get32(commit + 8) != get32(body + CRC_AT) ||
      get32(commit + 4) != get32(body + 16) || !valid_body(body, out))
    return PairKind::mismatch;
  return PairKind::committed;
}

struct BankScan {
  bool has_committed = false;
  bool any_nonfree = false;
  bool gap = false;       // data after free space
  bool mismatch = false;
  bool broken = false;    // sequence gap, slot regression or orphan body
  uint32_t tail = 0;      // index after the last non-free pair
  Image last;             // newest committed record of the bank
  uint32_t burnt[SLOTS] = {};  // highest next counters in uncommitted bodies after `last`
};

}  // namespace detail

class Journal {
 public:
  explicit Journal(Flash& flash) : flash_(flash) {}

  // Scans both banks. Call once at boot and again after Status::io_error.
  // Never writes; reads only.
  StorageState open() {
    using namespace detail;
    usable_ = false;
    torn_tail_ = false;
    image_ = Image{};
    active_ = 0;
    tail_ = 0;
    if (flash_.size() != REGION_BYTES) return StorageState::corrupt;
    BankScan scan[2];
    for (uint8_t bank = 0; bank < 2; ++bank)
      if (!scan_bank(bank, &scan[bank])) return StorageState::corrupt;

    if (!scan[0].has_committed && !scan[1].has_committed) {
      // Nothing was ever committed: blank, or an interrupted first write.
      if (scan[0].mismatch || scan[1].mismatch) return StorageState::corrupt;
      torn_tail_ = scan[0].any_nonfree || scan[1].any_nonfree;
      tail_ = scan[0].tail;
      usable_ = true;
      return state();
    }
    uint8_t active = 0;
    if (scan[0].has_committed && scan[1].has_committed) {
      if (scan[0].last.sequence == scan[1].last.sequence) return StorageState::corrupt;
      active = scan[1].last.sequence > scan[0].last.sequence ? 1 : 0;
    } else {
      active = scan[1].has_committed ? 1 : 0;
    }
    const BankScan& chosen = scan[active];
    if (chosen.gap || chosen.mismatch || chosen.broken) return StorageState::corrupt;
    active_ = active;
    tail_ = chosen.tail;
    image_ = chosen.last;
    for (uint8_t i = 0; i < SLOTS; ++i)  // burnt counters: never roll back
      if (image_.slots[i].state && chosen.burnt[i] > image_.slots[i].next)
        image_.slots[i].next = chosen.burnt[i];
    torn_tail_ = last_pair_torn(chosen);
    usable_ = true;
    return state();
  }

  StorageState state() const {
    if (!usable_) return StorageState::corrupt;
    if (!image_.generation) return StorageState::empty;
    return free_pairs() ? StorageState::ready : StorageState::full;
  }
  // True when open() skipped an uncommitted (interrupted) write.
  bool torn_tail_skipped() const { return torn_tail_; }

  // Journal generation as uppercase hex16; false (JSON null) when empty/corrupt.
  bool generation_hex(char out[17]) const {
    if (!usable_ || !image_.generation) return false;
    static const char hex_digits[] = "0123456789ABCDEF";
    for (int i = 0; i < 16; ++i) out[i] = hex_digits[(image_.generation >> (60 - 4 * i)) & 0xF];
    out[16] = '\0';
    return true;
  }

  Shutter shutter(uint8_t shutter_id) const {
    Shutter result;
    const detail::SlotRecord* s = slot(shutter_id);
    if (!s || !s->state) return result;
    result.shutter_id = shutter_id;
    result.state = static_cast<SlotState>(s->state);
    result.last_command = s->command;
    return result;
  }

  // PRIVATE raw RF identity of a provisioned slot, for TX and RX matching.
  bool identity(uint8_t shutter_id, uint32_t* out) const {
    const detail::SlotRecord* s = slot(shutter_id);
    if (!s || !s->state || !out) return false;
    *out = s->identity;
    return true;
  }
  // PRIVATE: trial admission can refuse a second enrollment even after
  // reboot or a burnt/torn reservation. Never export this counter over USB.
  bool next_counter(uint8_t shutter_id, uint32_t* out) const {
    const detail::SlotRecord* s = slot(shutter_id);
    if (!s || !s->state || !out) return false;
    *out = s->next;
    return true;
  }
  // PRIVATE: shutter_id owning this raw identity, 0 if none.
  uint8_t find_identity(uint32_t identity) const {
    if (!usable_ || !identity) return 0;
    for (uint8_t i = 0; i < SLOTS; ++i)
      if (image_.slots[i].state && image_.slots[i].identity == identity) return i + 1;
    return 0;
  }

  // Idempotent: an already provisioned slot is returned unchanged without a
  // write and without validating `fresh`. A new slot starts pending.
  Status provision(uint8_t shutter_id, const NewController& fresh,
                   Shutter* out = nullptr, bool* created = nullptr) {
    if (!usable_) return Status::corrupt;
    if (created) *created = false;
    if (!slot(shutter_id)) return Status::unknown_slot;
    if (image_.slots[shutter_id - 1].state) {
      if (out) *out = shutter(shutter_id);
      return Status::ok;
    }
    if (!fresh.identity || fresh.identity > 0xFFFFFF ||
        (!image_.generation && !fresh.generation))
      return Status::bad_argument;
    if (find_identity(fresh.identity)) return Status::identity_in_use;
    detail::Image next = image_;
    if (!next.generation) next.generation = fresh.generation;
    detail::SlotRecord& s = next.slots[shutter_id - 1];
    s.identity = fresh.identity;
    s.next = fresh.first_counter;
    s.state = static_cast<uint8_t>(SlotState::pending);
    s.command = 0;
    const Status status = append(next, false);
    if (status != Status::ok) return status;
    if (created) *created = true;
    if (out) *out = shutter(shutter_id);
    return Status::ok;
  }

  // pending -> paired, persisted. Already paired: ok without a write.
  Status confirm(uint8_t shutter_id, Shutter* out = nullptr) {
    if (!usable_) return Status::corrupt;
    const detail::SlotRecord* s = slot(shutter_id);
    if (!s || !s->state) return Status::unknown_slot;
    if (s->state != static_cast<uint8_t>(SlotState::paired)) {
      detail::Image next = image_;
      next.slots[shutter_id - 1].state = static_cast<uint8_t>(SlotState::paired);
      const Status status = append(next, false);
      if (status != Status::ok) return status;
    }
    if (out) *out = shutter(shutter_id);
    return Status::ok;
  }

  // Durably consumes one counter for one logical command (all radio copies
  // share it). Transmit only after Status::ok. An abandoned reservation stays
  // consumed. `command` is stored as last_command. critical (STOP) may use the
  // records kept in reserve and the last counter 0xFFFF; a movement is refused
  // with exhausted once next reaches 0xFFFF. Never erases.
  Status reserve(uint8_t shutter_id, uint8_t command, bool critical,
                 Reservation* out) {
    if (!usable_) return Status::corrupt;
    const detail::SlotRecord* s = slot(shutter_id);
    if (!s || !s->state) return Status::unknown_slot;
    if (!out) return Status::bad_argument;
    if (s->next >= MAX_NEXT || (!critical && s->next >= MAX_NEXT - 1))
      return Status::exhausted;
    const uint32_t counter = s->next;
    const uint32_t identity = s->identity;
    detail::Image next = image_;
    next.slots[shutter_id - 1].next = counter + 1;
    next.slots[shutter_id - 1].command = command;
    const Status status = append(next, critical);
    if (status != Status::ok) return status;
    out->shutter_id = shutter_id;
    out->identity = identity;
    out->counter = static_cast<uint16_t>(counter);
    return Status::ok;
  }

  // Call at idle. One bounded step per call (one sector erase or one record).
  bool maintenance_due() const {
    return usable_ && image_.generation && free_pairs() <= MAINTAIN_BELOW;
  }
  Status maintain() {
    using namespace detail;
    if (!usable_) return Status::corrupt;
    if (!maintenance_due()) return Status::ok;
    if (image_.sequence == UINT32_MAX) return Status::no_space;
    const uint8_t other = active_ ^ 1;
    for (uint32_t sector = 0; sector < SECTORS_PER_BANK; ++sector) {
      bool blank = false;
      if (!sector_erased(other, sector, &blank)) return fail();
      if (blank) continue;
      const uint32_t offset = other * BANK_BYTES + sector * SECTOR_BYTES;
      if (!flash_.erase_sector(offset)) return fail();
      if (!sector_erased(other, sector, &blank) || !blank) return fail();
      return Status::ok;
    }
    Image next = image_;
    next.sequence = image_.sequence + 1;
    const Status status = write_pair(other, 0, next);
    if (status != Status::ok) return status;
    active_ = other;
    tail_ = 1;
    image_ = next;
    return Status::ok;
  }

 private:
  uint32_t free_pairs() const { return tail_ < PAIRS_PER_BANK ? PAIRS_PER_BANK - tail_ : 0; }
  Status fail() {
    usable_ = false;
    return Status::io_error;
  }
  const detail::SlotRecord* slot(uint8_t shutter_id) const {
    return usable_ && shutter_id >= 1 && shutter_id <= SLOTS ? &image_.slots[shutter_id - 1] : nullptr;
  }
  static uint32_t pair_offset(uint8_t bank, uint32_t pair) {
    return bank * BANK_BYTES + pair * 2 * PAGE_BYTES;
  }
  bool read_pair(uint8_t bank, uint32_t pair) {
    const uint32_t offset = pair_offset(bank, pair);
    return flash_.read(offset, body_, PAGE_BYTES) &&
           flash_.read(offset + PAGE_BYTES, commit_, PAGE_BYTES);
  }
  bool scan_bank(uint8_t bank, detail::BankScan* scan) {
    detail::Image image;
    bool seen_free = false;
    for (uint32_t pair = 0; pair < PAIRS_PER_BANK; ++pair) {
      if (!read_pair(bank, pair)) return false;
      const detail::PairKind kind = detail::classify(body_, commit_, &image);
      if (kind == detail::PairKind::free) {
        seen_free = true;
        continue;
      }
      scan->any_nonfree = true;
      scan->tail = pair + 1;
      if (seen_free) scan->gap = true;
      if (kind == detail::PairKind::mismatch) scan->mismatch = true;
      if (kind == detail::PairKind::torn && scan->has_committed &&
          detail::valid_body(body_, &image)) {
        // Intact body, missing/damaged commit: its counters may have been
        // emitted. It must be the direct successor of the newest commit.
        if (image.sequence != scan->last.sequence + 1 ||
            !detail::valid_transition(scan->last, image))
          scan->broken = true;
        for (uint8_t i = 0; i < SLOTS; ++i)
          if (image.slots[i].next > scan->burnt[i]) scan->burnt[i] = image.slots[i].next;
      }
      if (kind != detail::PairKind::committed) continue;
      if (scan->has_committed &&
          (image.sequence != scan->last.sequence + 1 ||
           !detail::valid_transition(scan->last, image)))
        scan->broken = true;
      scan->last = image;
      scan->has_committed = true;
      memset(scan->burnt, 0, sizeof(scan->burnt));  // superseded by this commit
    }
    return true;
  }
  // True when the bank's final non-free pair is not the newest commit.
  bool last_pair_torn(const detail::BankScan& scan) {
    detail::Image ignored;
    if (!scan.tail || !read_pair(active_, scan.tail - 1)) return false;
    return detail::classify(body_, commit_, &ignored) == detail::PairKind::torn;
  }
  bool sector_erased(uint8_t bank, uint32_t sector, bool* blank) {
    *blank = true;
    for (uint32_t page = 0; page < SECTOR_BYTES / PAGE_BYTES; ++page) {
      if (!flash_.read(bank * BANK_BYTES + sector * SECTOR_BYTES + page * PAGE_BYTES,
                       check_, PAGE_BYTES))
        return false;
      if (!detail::erased(check_, PAGE_BYTES)) *blank = false;
    }
    return true;
  }
  bool program_verified(uint32_t offset, const uint8_t* page) {
    return flash_.program_page(offset, page) &&
           flash_.read(offset, check_, PAGE_BYTES) &&
           memcmp(check_, page, PAGE_BYTES) == 0;
  }
  // Body first, then the commit page once the body reads back correctly.
  Status write_pair(uint8_t bank, uint32_t pair, const detail::Image& image) {
    detail::encode_body(image, body_);
    detail::encode_commit(image, body_, commit_);
    const uint32_t offset = pair_offset(bank, pair);
    if (!program_verified(offset, body_) ||
        !program_verified(offset + PAGE_BYTES, commit_))
      return fail();
    return Status::ok;
  }
  Status append(detail::Image next, bool critical) {
    if (!usable_) return Status::corrupt;
    // tail_ is the first pair after the last non-free one, so it is erased.
    if (image_.sequence == UINT32_MAX || tail_ >= PAIRS_PER_BANK ||
        PAIRS_PER_BANK - tail_ <= (critical ? 0u : CRITICAL_PAIRS))
      return Status::no_space;
    next.sequence = image_.sequence + 1;
    const Status status = write_pair(active_, tail_, next);
    if (status != Status::ok) return status;
    ++tail_;
    image_ = next;
    return Status::ok;
  }

  Flash& flash_;
  detail::Image image_;
  uint8_t active_ = 0;
  uint32_t tail_ = 0;
  bool usable_ = false;
  bool torn_tail_ = false;
  uint8_t body_[PAGE_BYTES], commit_[PAGE_BYTES], check_[PAGE_BYTES];
};

}  // namespace journal
}  // namespace ha_x2d
