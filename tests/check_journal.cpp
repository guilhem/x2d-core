// Lifecycle persistence and power-cut checks. All identities are synthetic;
// this does not qualify a motor profile or a hardware flash backend.
#include <x2d/journal.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <array>
#include <vector>

using namespace x2d::journal;
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"check_journal:%d: %s\n",__LINE__,#x); abort(); } } while (0)
constexpr uint64_t GENERATION=0x123456789ABCDEF0ull;
static unsigned cut_checks=0;
static void settle(Journal& journal) {
  for (unsigned step=0; journal.maintenance_due(); ++step) {
    CHECK(step<40); CHECK(journal.maintain()==Status::ok);
  }
}
static void initialize(Journal& journal, uint16_t seed=0xCAFE) {
  CHECK(journal.open()==StorageState::empty);
  CHECK(!journal.enrollment_profile_matches(1));
  CHECK(journal.initialize(GENERATION,seed)==Status::ok);
  CHECK(journal.state()==StorageState::initializing);
  Reservation reservation;
  CHECK(journal.reserve(1,1,false,&reservation)==Status::not_initialized);
  settle(journal); CHECK(journal.state()==StorageState::ready);
  CHECK(journal.enrollment_profile_matches(1) && !journal.enrollment_profile_matches(90));
}
static void attempt(Journal& journal, uint8_t slot, bool retry=false) {
  settle(journal); CHECK(journal.claim_attempt(slot,retry)==Status::ok);
  const auto epoch=journal.incarnation(slot,true);
  CHECK(epoch);
  Reservation first, second;
  CHECK(journal.reserve(slot,0,false,&first,epoch,true)==Status::ok);
  CHECK(journal.reserve(slot,0,false,&second,epoch,true)==Status::ok);
  CHECK(first.incarnation==epoch && second.incarnation==epoch);
  CHECK(first.identity==second.identity && first.counter==(retry?2:0) && second.counter==first.counter+1);
}
static void add(Journal& journal, uint8_t slot) {
  settle(journal); CHECK(journal.allocate_candidate(slot,false)==Status::ok);
  attempt(journal,slot); CHECK(journal.confirm_candidate(slot)==Status::ok);
  settle(journal);
}
static std::vector<uint8_t> bytes(MemoryFlash& flash) { return {flash.raw(),flash.raw()+REGION_BYTES}; }
static detail::Image newest(MemoryFlash& flash, uint32_t* offset=nullptr) {
  detail::Image best;
  for (uint32_t bank=0; bank<2; ++bank) for (uint32_t i=0; i<PAIRS_PER_BANK; ++i) {
    const uint32_t at=bank*BANK_BYTES+i*RECORD_BYTES; detail::Image image;
    if (detail::classify(flash.raw()+at,flash.raw()+at+BODY_BYTES,&image)==detail::PairKind::committed && image.sequence>best.sequence) {
      best=image; if (offset) *offset=at;
    }
  }
  CHECK(best.sequence); return best;
}
static void append_fixture(MemoryFlash& flash, detail::Image image) {
  uint32_t offset; const auto old=newest(flash,&offset);
  CHECK(offset%BANK_BYTES+2*RECORD_BYTES<=FENCE_AT);
  image.sequence=old.sequence+1;
  std::array<uint8_t,BODY_BYTES> body; std::array<uint8_t,PAGE_BYTES> commit;
  detail::encode_body(image,body.data()); detail::encode_commit(image,body.data(),commit.data());
  CHECK(flash.program_page(offset+RECORD_BYTES,body.data()));
  CHECK(flash.program_page(offset+RECORD_BYTES+PAGE_BYTES,body.data()+PAGE_BYTES));
  CHECK(flash.program_page(offset+RECORD_BYTES+BODY_BYTES,commit.data()));
}
static void legacy_record(MemoryFlash& flash,uint8_t bank,uint32_t index,uint32_t seq,uint32_t first=0xCAFE01,uint32_t second=0xCAFF01) {
  uint8_t body[PAGE_BYTES],commit[PAGE_BYTES]; memset(body,0xFF,sizeof(body)); memset(commit,0xFF,sizeof(commit));
  detail::put32(body,detail::JOURNAL_MAGIC); detail::put16(body+4,1); detail::put16(body+6,SLOTS);
  detail::put64(body+8,GENERATION); detail::put32(body+16,seq); detail::put32(body+20,0);
  for (uint8_t i=0; i<SLOTS; ++i) {
    auto* slot=body+24+i*12; detail::put32(slot,i==0?first:i==1?second:0);
    detail::put32(slot+4,i<2?200+seq:0); slot[8]=i<2?2:0; slot[9]=0; slot[10]=slot[11]=0;
  }
  detail::put32(body+252,detail::crc32(body,252));
  detail::put32(commit,detail::COMMIT_MAGIC); detail::put32(commit+4,seq);
  detail::put32(commit+8,detail::get32(body+252)); detail::put32(commit+12,detail::crc32(commit,12));
  const auto at=bank*BANK_BYTES+index*2*PAGE_BYTES;
  CHECK(flash.program_page(at,body)); CHECK(flash.program_page(at+PAGE_BYTES,commit));
}
// v1's no-committed-record path must find a mismatching valid commit, rather
// than return empty. This independently examines v1's 512-byte geometry.
static bool old_reader_rejects(MemoryFlash& flash) {
  bool mismatch=false, committed=false;
  for (uint32_t at=0; at<REGION_BYTES; at+=2*PAGE_BYTES) {
    const auto* body=flash.raw()+at; const auto* commit=body+PAGE_BYTES;
    if (detail::get32(commit)!=detail::COMMIT_MAGIC || detail::get32(commit+12)!=detail::crc32(commit,12)) continue;
    if (detail::get32(body)!=detail::JOURNAL_MAGIC || detail::get16(body+4)!=1 ||
        detail::get32(body+252)!=detail::crc32(body,252) || detail::get32(commit+8)!=detail::get32(body+252)) mismatch=true;
    else committed=true;
  }
  return mismatch && !committed;
}

static void lifecycle() {
  MemoryFlash flash; Journal journal(flash); initialize(journal);
  CHECK(journal.allocate_candidate(0,false)==Status::unknown_slot);
  CHECK(journal.allocate_candidate(1,false)==Status::ok);
  const auto pending=journal.shutter(1);
  CHECK(pending.state==SlotState::pending && pending.logical_id==2 && pending.has_candidate && !pending.in_service);
  CHECK(journal.allocate_candidate(2,false)==Status::busy);
  CHECK(journal.confirm_candidate(1)==Status::no_attempt);
  attempt(journal,1);
  CHECK(journal.claim_attempt(1,false)==Status::retry_unavailable);
  attempt(journal,1,true);
  CHECK(journal.claim_attempt(1,true)==Status::retry_unavailable);
  CHECK(journal.confirm_candidate(1)==Status::ok);
  const auto initial=journal.shutter(1); uint32_t old_identity,old_next;
  CHECK(initial.state==SlotState::paired && initial.in_service && !initial.has_candidate);
  CHECK(journal.identity(1,&old_identity) && journal.next_counter(1,&old_next) && old_next==4);
  CHECK(journal.retire(1)==Status::disabled);
  CHECK(journal.allocate_candidate(1,true)==Status::disabled);
  CHECK(journal.set_service(1,false)==Status::ok);
  CHECK(journal.allocate_candidate(1,true)==Status::ok);
  const auto replacing=journal.shutter(1);
  CHECK(replacing.logical_id==initial.logical_id && replacing.incarnation==initial.incarnation && replacing.replacement);
  CHECK(journal.identity(1,&old_next) && old_next==old_identity);
  CHECK(journal.retire(1)==Status::busy && journal.set_service(1,true)==Status::busy);
  attempt(journal,1);
  const uint32_t burned=journal.incarnation(1,true);
  CHECK(journal.cancel_candidate(1)==Status::ok);
  CHECK(journal.shutter(1).incarnation==initial.incarnation && !journal.shutter(1).in_service);
  settle(journal); CHECK(journal.allocate_candidate(1,true)==Status::ok);
  CHECK(journal.incarnation(1,true)>burned);
  attempt(journal,1); CHECK(journal.confirm_candidate(1)==Status::ok);
  CHECK(journal.shutter(1).logical_id==initial.logical_id && journal.shutter(1).incarnation>initial.incarnation);
  Reservation reservation; const auto before=flash.mutations();
  CHECK(journal.reserve(1,1,false,&reservation,initial.incarnation)==Status::incarnation_mismatch);
  CHECK(flash.mutations()==before);
  CHECK(journal.set_service(1,false)==Status::ok && journal.retire(1)==Status::ok);
  CHECK(journal.shutter(1).logical_id==0 && !journal.identity(1,&old_identity));
  CHECK(journal.reserve(1,1,true,&reservation,initial.incarnation)==Status::unknown_slot);
  settle(journal); CHECK(journal.allocate_candidate(1,false)==Status::ok);
  CHECK(journal.shutter(1).logical_id==3);
  CHECK(journal.cancel_candidate(1)==Status::ok && journal.shutter(1).state==SlotState::unused);
  CHECK(journal.allocate_candidate(1,false)==Status::ok && journal.shutter(1).logical_id==4);
  Journal booted(flash); CHECK(booted.open()==StorageState::ready);
  CHECK(booted.shutter(1).has_candidate && booted.shutter(1).attempts==0);
  CHECK(booted.confirm_candidate(1)==Status::no_attempt && booted.claim_attempt(1,true)==Status::retry_unavailable);
  CHECK(old_reader_rejects(flash) && flash.violations()==0);
}
static void capacity_and_history() {
  MemoryFlash flash; Journal journal(flash); initialize(journal,65535);
  for (uint8_t slot=1; slot<=SLOTS; ++slot) add(journal,slot);
  CHECK(journal.allocate_candidate(1,false)==Status::inventory_full);
  CHECK(journal.set_service(1,false)==Status::ok && journal.retire(1)==Status::ok);
  add(journal,1); CHECK(journal.shutter(1).logical_id==18);
  uint32_t id; CHECK(journal.identity(2,&id) && id==0x000101); // zero prefix skipped without wrapping cursor
  for (unsigned i=0; i<40; ++i) {
    CHECK(journal.set_service(1,false)==Status::ok && journal.retire(1)==Status::ok); add(journal,1);
  }
  Journal boot(flash); CHECK(boot.open()==StorageState::ready && boot.shutter(1).logical_id==58);
  CHECK(old_reader_rejects(flash));
  // The trusted primitives retain arbitrary keys in their exclusion history.
  MemoryFlash primitive; Journal p(primitive); CHECK(p.open()==StorageState::empty);
  CHECK(p.provision(1,{0x123401,10,GENERATION})==Status::ok && p.confirm(1)==Status::ok);
  CHECK(p.set_service(1,false)==Status::ok && p.retire(1)==Status::ok);
  CHECK(p.provision(1,{0x123401,0,GENERATION})==Status::identity_in_use);
  Journal again(primitive); CHECK(again.open()==StorageState::ready);
  CHECK(again.provision(2,{0x123401,0,GENERATION})==Status::identity_in_use);
}
static void bounds() {
  MemoryFlash flash; Journal journal(flash); initialize(journal);
  for (unsigned logical=2; logical<=254; ++logical) {
    settle(journal); CHECK(journal.allocate_candidate(1,false)==Status::ok);
    CHECK(journal.shutter(1).logical_id==logical && journal.cancel_candidate(1)==Status::ok);
  }
  settle(journal); auto before=flash.mutations();
  CHECK(journal.allocate_candidate(1,false)==Status::logical_exhausted && flash.mutations()==before);
  MemoryFlash exhausted; Journal j(exhausted); initialize(j);
  auto image=newest(exhausted); image.prefix_cursor=RF_PREFIXES; append_fixture(exhausted,image);
  Journal reopened(exhausted); CHECK(reopened.open()==StorageState::ready); before=exhausted.mutations();
  CHECK(reopened.allocate_candidate(1,false)==Status::identity_exhausted && exhausted.mutations()==before);
}
static void reset_v1() {
  MemoryFlash flash; legacy_record(flash,0,0,1); legacy_record(flash,0,1,2); legacy_record(flash,1,0,1);
  Journal journal(flash); const auto before=bytes(flash);
  CHECK(journal.open()==StorageState::legacy && bytes(flash)==before);
  Reservation reservation; CHECK(journal.reserve(1,1,true,&reservation)==Status::not_initialized);
  CHECK(journal.initialize(GENERATION+1,0xCAFE)==Status::ok && journal.state()==StorageState::initializing);
  CHECK(journal.allocate_candidate(1,false)==Status::not_initialized);
  settle(journal); CHECK(old_reader_rejects(flash));
  CHECK(journal.allocate_candidate(1,false)==Status::ok);
  uint32_t identity; CHECK(journal.identity(1,&identity,true) && identity==0xCB0001);
  CHECK(journal.shutter(1).logical_id==2 && journal.cancel_candidate(1)==Status::ok);
  CHECK(journal.provision(1,{0xCAFE01,0,GENERATION})==Status::identity_in_use);
  Journal reboot(flash); CHECK(reboot.open()==StorageState::ready);
  CHECK(reboot.provision(1,{0xCAFF01,0,GENERATION})==Status::identity_in_use);
  // Bringing back an old v1 bank must never quietly restart its stale binding.
  MemoryFlash stale; memcpy(stale.raw(),flash.raw(),REGION_BYTES);
  const auto active=newest(stale); (void)active;
  memset(stale.raw(),0xFF,BANK_BYTES); legacy_record(stale,0,0,1);
  Journal regression(stale); CHECK(regression.open()==StorageState::corrupt);
  const auto snapshot=bytes(stale); CHECK(regression.initialize(5,1)==Status::corrupt && bytes(stale)==snapshot);
}

template<class Operation,class Verify>
static void cut_matrix(MemoryFlash& source, Operation operation, Verify verify, unsigned mutations=3) {
  const auto snapshot=bytes(source);
  const unsigned tears[]={0,1,4,8,24,100,252,255,256};
  for (unsigned cut=0; cut<mutations; ++cut) for (unsigned tear:tears) {
    MemoryFlash flash; memcpy(flash.raw(),snapshot.data(),REGION_BYTES);
    Journal journal(flash); CHECK(journal.open()!=StorageState::corrupt);
    flash.cut_after(cut,tear); const auto result=operation(journal);
    CHECK(result==Status::io_error && !flash.powered());
    flash.restore_power(); Journal recovered(flash); const auto state=recovered.open();
    if (state==StorageState::corrupt) fprintf(stderr,"cut=%u tear=%u unexpected corruption\n",cut,tear);
    CHECK(state!=StorageState::corrupt); verify(recovered,flash,cut,tear);
    CHECK(flash.violations()==0); ++cut_checks;
  }
}
static void initialization_cuts() {
  MemoryFlash blank;
  cut_matrix(blank,[](Journal& j){return j.initialize(GENERATION,0xCAFE);},
    [](Journal& j,MemoryFlash& f,unsigned,unsigned){
      if (j.state()==StorageState::empty) CHECK(j.initialize(GENERATION,0xCAFE)==Status::ok);
      settle(j); CHECK(j.state()==StorageState::ready && old_reader_rejects(f));
    });
  MemoryFlash source; Journal j(source); CHECK(j.open()==StorageState::empty && j.initialize(GENERATION,0xCAFE)==Status::ok);
  // Interrupt either fence page, then recover without touching live records.
  cut_matrix(source,[](Journal& journal){return journal.maintain();},
    [](Journal& journal,MemoryFlash& f,unsigned,unsigned){settle(journal); CHECK(old_reader_rejects(f));},2);
  CHECK(j.maintain()==Status::ok);
  cut_matrix(source,[](Journal& journal){return journal.maintain();},
    [](Journal& journal,MemoryFlash& f,unsigned,unsigned){settle(journal); CHECK(old_reader_rejects(f));});
}
static void allocation_cuts() {
  MemoryFlash source; Journal journal(source); initialize(journal);
  cut_matrix(source,[](Journal& j){return j.allocate_candidate(1,false);},
    [](Journal& j,MemoryFlash&,unsigned cut,unsigned tear){
      if (j.shutter(1).has_candidate) { CHECK(j.shutter(1).logical_id==2 && j.shutter(1).attempts==0); CHECK(j.cancel_candidate(1)==Status::ok); }
      CHECK(j.allocate_candidate(1,false)==Status::ok);
      // A complete body with no valid commit burns both logical and RF IDs.
      if (cut==2) CHECK(j.shutter(1).logical_id==3 && j.incarnation(1,true)==2);
      (void)tear;
    });
}
static void attempt_and_reservation_cuts() {
  MemoryFlash source; Journal journal(source); initialize(journal);
  CHECK(journal.allocate_candidate(1,false)==Status::ok);
  cut_matrix(source,[](Journal& j){return j.claim_attempt(1,false);},
    [](Journal& j,MemoryFlash&,unsigned cut,unsigned){
      if (cut==2) CHECK(j.shutter(1).attempts==1 && j.claim_attempt(1,false)==Status::retry_unavailable);
      CHECK(j.claim_attempt(1,true)==Status::retry_unavailable);
    });
  CHECK(journal.claim_attempt(1,false)==Status::ok);
  for (unsigned phase=0; phase<2; ++phase) {
    cut_matrix(source,[](Journal& j){Reservation r; return j.reserve(1,0,false,&r,j.incarnation(1,true),true);},
      [phase](Journal& j,MemoryFlash&,unsigned cut,unsigned){
        uint32_t next; CHECK(j.next_counter(1,&next,true) && next>=phase);
        if (cut==2) CHECK(next==phase+1);
        if (next==1) CHECK(j.claim_attempt(1,true)==Status::retry_unavailable);
        CHECK(j.shutter(1).attempts==1);
      });
    Reservation r; CHECK(journal.reserve(1,0,false,&r,journal.incarnation(1,true),true)==Status::ok);
  }
  cut_matrix(source,[](Journal& j){return j.claim_attempt(1,true);},
    [](Journal& j,MemoryFlash&,unsigned cut,unsigned){
      if (cut==2) CHECK(j.shutter(1).attempts==2 && j.claim_attempt(1,true)==Status::retry_unavailable);
    });
}
static void replacement_and_retirement_cuts() {
  MemoryFlash source; Journal journal(source); initialize(journal); add(journal,1);
  const auto initial=journal.shutter(1); CHECK(journal.set_service(1,false)==Status::ok);
  cut_matrix(source,[](Journal& j){return j.allocate_candidate(1,true);},
    [initial](Journal& j,MemoryFlash&,unsigned cut,unsigned){
      CHECK(j.shutter(1).logical_id==initial.logical_id && j.incarnation(1)==initial.incarnation && !j.shutter(1).in_service);
      if (j.shutter(1).has_candidate) CHECK(j.cancel_candidate(1)==Status::ok);
      CHECK(j.allocate_candidate(1,true)==Status::ok);
      if (cut==2) CHECK(j.incarnation(1,true)>initial.incarnation+1);
    });
  CHECK(journal.allocate_candidate(1,true)==Status::ok); attempt(journal,1);
  cut_matrix(source,[](Journal& j){return j.confirm_candidate(1);},
    [initial](Journal& j,MemoryFlash&,unsigned,unsigned){
      const auto s=j.shutter(1); CHECK(s.logical_id==initial.logical_id);
      if (s.has_candidate) { CHECK(s.incarnation==initial.incarnation && !s.in_service); CHECK(j.confirm_candidate(1)==Status::ok); }
      CHECK(j.incarnation(1)>initial.incarnation);
      Reservation r; CHECK(j.reserve(1,1,false,&r,initial.incarnation)==Status::incarnation_mismatch);
    });
  cut_matrix(source,[](Journal& j){return j.cancel_candidate(1);},
    [initial](Journal& j,MemoryFlash&,unsigned,unsigned){CHECK(j.incarnation(1)==initial.incarnation && !j.shutter(1).in_service);});
  CHECK(journal.cancel_candidate(1)==Status::ok);
  cut_matrix(source,[](Journal& j){return j.retire(1);},
    [](Journal& j,MemoryFlash&,unsigned cut,unsigned){
      CHECK(!j.shutter(1).in_service);
      if (cut==2) CHECK(j.shutter(1).logical_id==0);
      CHECK(j.retire(1)==Status::ok); settle(j);
      CHECK(j.allocate_candidate(1,false)==Status::ok && j.shutter(1).logical_id==3);
    });
}
static void stop_budget() {
  MemoryFlash flash; Journal journal(flash); initialize(journal);
  for (uint8_t slot=1; slot<=SLOTS; ++slot) add(journal,slot);
  Reservation reservation; unsigned after_due=0;
  for (;;) {
    const bool due=journal.maintenance_due();
    const auto result=journal.reserve(1,1,false,&reservation,journal.incarnation(1));
    if (result==Status::no_space) break;
    CHECK(result==Status::ok); if (due) ++after_due;
  }
  CHECK(after_due==16);
  const auto erases=flash.erases();
  for (uint8_t slot=1; slot<=SLOTS; ++slot) CHECK(journal.reserve(slot,3,true,&reservation,journal.incarnation(slot))==Status::ok);
  CHECK(flash.erases()==erases && journal.state()==StorageState::full);
  CHECK(journal.reserve(1,3,true,&reservation,journal.incarnation(1))==Status::no_space);
  settle(journal); CHECK(journal.reserve(1,3,true,&reservation,journal.incarnation(1))==Status::ok);
  CHECK(old_reader_rejects(flash));
}
static void corruption() {
  MemoryFlash source; Journal journal(source); initialize(journal); add(journal,1);
  Reservation r; CHECK(journal.reserve(1,1,false,&r,journal.incarnation(1))==Status::ok);
  uint32_t at; newest(source,&at);
  for (uint32_t byte: {0u,100u,256u,400u,508u}) {
    MemoryFlash flash; memcpy(flash.raw(),source.raw(),REGION_BYTES); flash.raw()[at+byte]^=1;
    Journal damaged(flash); CHECK(damaged.open()==StorageState::corrupt);
    const auto snapshot=bytes(flash); CHECK(damaged.initialize(1,1)==Status::corrupt && damaged.maintain()==Status::corrupt && bytes(flash)==snapshot);
  }
  // A damaged newest commit with its full body intact burns the emitted counter.
  MemoryFlash burnt; memcpy(burnt.raw(),source.raw(),REGION_BYTES); burnt.raw()[at+BODY_BYTES]^=1;
  Journal j(burnt); CHECK(j.open()==StorageState::ready);
  CHECK(j.reserve(1,1,false,&r,j.incarnation(1))==Status::ok && r.counter==3);
  MemoryFlash unknown; unknown.raw()[0]=0; Journal refused(unknown); const auto raw=bytes(unknown);
  CHECK(refused.open()==StorageState::corrupt && refused.initialize(1,1)==Status::corrupt && bytes(unknown)==raw);
}
static void invalid_initializer_is_not_empty() {
  MemoryFlash source; Journal j(source); CHECK(j.open()==StorageState::empty);
  CHECK(j.initialize(GENERATION,0xCAFE)==Status::ok);
  for (bool unsupported: {false,true}) {
    MemoryFlash f; memcpy(f.raw(),source.raw(),REGION_BYTES);
    memset(f.raw()+BODY_BYTES,0xFF,PAGE_BYTES); // no valid commit
    if (unsupported) detail::put16(f.raw()+4,0xFFFF);
    else f.raw()[detail::SLOTS_AT+13]=uint8_t(SlotState::paired);
    Journal damaged(f); const auto snapshot=bytes(f);
    CHECK(damaged.open()==StorageState::corrupt && damaged.initialize(1,1)==Status::corrupt && bytes(f)==snapshot);
  }
}
static void private_profile_policy() {
  MemoryFlash flash; Journal j(flash);
  CHECK(j.open()==StorageState::empty && !j.enrollment_profile_matches(90));
  CHECK(j.initialize(GENERATION,0xCAFE,90)==Status::ok);
  CHECK(!j.enrollment_profile_matches(90)); settle(j);
  CHECK(j.enrollment_profile_matches(90) && !j.enrollment_profile_matches(1));
  add(j,1); uint32_t identity;
  CHECK(j.identity(1,&identity) && uint8_t(identity)==90);
  Journal boot(flash); CHECK(boot.open()==StorageState::ready && boot.enrollment_profile_matches(90));
  CHECK(!boot.enrollment_profile_matches(1));
}
static void primitive_bounds() {
  MemoryFlash flash; Journal j(flash);
  CHECK(j.state()==StorageState::corrupt && j.open()==StorageState::empty);
  const auto before=flash.mutations();
  CHECK(j.initialize(0,1)==Status::bad_argument && flash.mutations()==before);
  CHECK(j.provision(1,{0,0,GENERATION})==Status::bad_argument);
  CHECK(j.provision(1,{0x1000000,0,GENERATION})==Status::bad_argument);
  CHECK(j.provision(0,{1,0,GENERATION})==Status::unknown_slot);
  CHECK(j.provision(17,{1,0,GENERATION})==Status::unknown_slot);
  CHECK(j.provision(1,{0x111111,0xFFFD,GENERATION})==Status::ok);
  Shutter shutter; bool created=true; const auto writes=flash.mutations();
  CHECK(j.provision(1,{0,9,0},&shutter,&created)==Status::ok && !created && flash.mutations()==writes);
  CHECK(j.confirm(1)==Status::ok);
  CHECK(j.confirm(1,&shutter)==Status::ok && shutter.in_service);
  Reservation r;
  CHECK(j.reserve(1,1,false,&r,shutter.incarnation)==Status::ok && r.counter==0xFFFD);
  CHECK(j.reserve(1,1,false,&r,shutter.incarnation)==Status::ok && r.counter==0xFFFE);
  CHECK(j.reserve(1,1,false,&r,shutter.incarnation)==Status::exhausted);
  CHECK(j.reserve(1,3,true,&r,shutter.incarnation)==Status::ok && r.counter==0xFFFF);
  Journal reboot(flash); CHECK(reboot.open()==StorageState::ready);
  CHECK(reboot.reserve(1,3,true,&r,reboot.incarnation(1))==Status::exhausted);
  struct Small : Flash {
    uint32_t size() const override { return 4096; }
    bool read(uint32_t,void*,uint32_t) override { return false; }
    bool erase_sector(uint32_t) override { return false; }
    bool program_page(uint32_t,const uint8_t*) override { return false; }
  } small;
  Journal wrong(small); CHECK(wrong.open()==StorageState::corrupt);
}
static void malformed_snapshots() {
  MemoryFlash source; Journal j(source); initialize(j); add(j,1);
  uint32_t offset; const auto original=newest(source,&offset);
  for (unsigned change=0; change<8; ++change) {
    MemoryFlash f; memcpy(f.raw(),source.raw(),REGION_BYTES);
    auto image=original;
    switch (change) {
      case 0: image.slots[0].next=1; break;
      case 1: image.slots[0].identity=0xF00001; break;
      case 2: image.slots[0].incarnation+=5; break;
      case 3: image.next_logical=2; break;
      case 4: image.prefix_cursor=0; break;
      case 5: image.next_incarnation=1; break;
      case 6: image.slots[0].logical_id=254; break;
      case 7: image.slots[0].state=uint8_t(SlotState::pending); break;
    }
    append_fixture(f,image); Journal rejected(f); const auto raw=bytes(f);
    CHECK(rejected.open()==StorageState::corrupt && rejected.initialize(1,1)==Status::corrupt && bytes(f)==raw);
  }
  // A supported-looking CRC cannot turn an unknown version into empty flash.
  MemoryFlash unknown; memcpy(unknown.raw(),source.raw(),REGION_BYTES);
  detail::put16(unknown.raw()+offset+4,3);
  detail::put32(unknown.raw()+offset+detail::CRC_AT,detail::crc32(unknown.raw()+offset,detail::CRC_AT));
  detail::put32(unknown.raw()+offset+BODY_BYTES+8,detail::get32(unknown.raw()+offset+detail::CRC_AT));
  detail::put32(unknown.raw()+offset+BODY_BYTES+12,detail::crc32(unknown.raw()+offset+BODY_BYTES,12));
  Journal bad(unknown); const auto raw=bytes(unknown);
  CHECK(bad.open()==StorageState::corrupt && bad.initialize(1,1)==Status::corrupt && bytes(unknown)==raw);
  // Sequence gaps and data after a genuinely free record are corruption.
  for (bool gap: {false,true}) {
    MemoryFlash f; memcpy(f.raw(),source.raw(),REGION_BYTES);
    if (gap) f.raw()[offset+3*RECORD_BYTES]=0;
    else { auto next=original; next.sequence+=5; uint8_t body[BODY_BYTES],commit[PAGE_BYTES];
      detail::encode_body(next,body); detail::encode_commit(next,body,commit);
      memcpy(f.raw()+offset+RECORD_BYTES,body,BODY_BYTES); memcpy(f.raw()+offset+RECORD_BYTES+BODY_BYTES,commit,PAGE_BYTES); }
    Journal broken(f); CHECK(broken.open()==StorageState::corrupt);
  }
}
// A broken/new-format rotation record may be the newest durable write.
// Never fall back to the older bank even when that older image is valid.
static void newer_bank_corruption() {
  MemoryFlash source; Journal j(source); initialize(j); add(j,1);
  Reservation r;
  while (!j.maintenance_due()) CHECK(j.reserve(1,1,false,&r,j.incarnation(1))==Status::ok);
  settle(j);
  uint32_t offset; const auto image=newest(source,&offset);
  CHECK(offset%BANK_BYTES==0);
  for (bool unknown: {false,true}) {
    MemoryFlash f; memcpy(f.raw(),source.raw(),REGION_BYTES);
    if (unknown) {
      detail::put16(f.raw()+offset+4,3);
      detail::put32(f.raw()+offset+detail::CRC_AT,detail::crc32(f.raw()+offset,detail::CRC_AT));
      detail::put32(f.raw()+offset+BODY_BYTES+8,detail::get32(f.raw()+offset+detail::CRC_AT));
      detail::put32(f.raw()+offset+BODY_BYTES+12,detail::crc32(f.raw()+offset+BODY_BYTES,12));
    } else f.raw()[offset+100]^=1;
    Journal damaged(f); const auto snapshot=bytes(f);
    CHECK(damaged.open()==StorageState::corrupt && damaged.initialize(1,1)==Status::corrupt && bytes(f)==snapshot);
  }
  CHECK(image.sequence);
}
static void full_bit_flip_sweep() {
  MemoryFlash source; Journal j(source); initialize(j); add(j,1);
  Reservation r; CHECK(j.reserve(1,1,false,&r,j.incarnation(1))==Status::ok && r.counter==2);
  uint32_t at; newest(source,&at);
  unsigned corrupt=0,burnt=0;
  for (unsigned byte=0; byte<RECORD_BYTES; ++byte) for (unsigned bit=0; bit<8; ++bit) {
    MemoryFlash f; memcpy(f.raw(),source.raw(),REGION_BYTES); f.raw()[at+byte]^=uint8_t(1u<<bit);
    Journal recovered(f);
    if (recovered.open()==StorageState::corrupt) { ++corrupt; continue; }
    CHECK(recovered.reserve(1,1,false,&r,recovered.incarnation(1))==Status::ok && r.counter==3); ++burnt;
  }
  CHECK(corrupt==BODY_BYTES*8 && burnt==PAGE_BYTES*8);
  printf("journal bit flips: %u corrupt, %u safe commit outcomes\n",corrupt,burnt);
}
static bool reservation_scenario(Journal& j, int64_t& highest, unsigned count) {
  for (unsigned i=0; i<count; ++i) {
    while (j.maintenance_due()) if (j.maintain()==Status::io_error) return false;
    Reservation r; const auto result=j.reserve(1,1,false,&r,j.incarnation(1));
    if (result==Status::io_error) return false;
    CHECK(result==Status::ok && r.counter>highest); highest=r.counter;
  }
  return true;
}
static void rotation_power_cuts() {
  MemoryFlash source; Journal ready(source); initialize(ready); add(ready,1);
  const auto source_bytes=bytes(source);
  MemoryFlash baseline; memcpy(baseline.raw(),source_bytes.data(),REGION_BYTES);
  Journal j(baseline); CHECK(j.open()==StorageState::ready); int64_t highest=1;
  CHECK(reservation_scenario(j,highest,36)); const auto mutations=baseline.mutations();
  CHECK(baseline.erases()>=4); // at least two rotations erased previously used sectors
  const unsigned tears[]={0,4,100,255,256,4096};
  for (unsigned cut=0; cut<mutations; ++cut) for (unsigned tear:tears) {
    MemoryFlash f; memcpy(f.raw(),source_bytes.data(),REGION_BYTES);
    Journal running(f); CHECK(running.open()==StorageState::ready); highest=1;
    f.cut_after(cut,tear); CHECK(!reservation_scenario(running,highest,36)); f.restore_power();
    Journal reboot(f); const auto state=reboot.open();
    if (state==StorageState::corrupt) fprintf(stderr,"rotation cut=%u tear=%u highest=%lld\n",cut,tear,(long long)highest);
    CHECK(state!=StorageState::corrupt && reboot.shutter(1).in_service);
    CHECK(reservation_scenario(reboot,highest,4)); CHECK(f.violations()==0 && old_reader_rejects(f)); ++cut_checks;
  }
}
static void legacy_reset_cuts() {
  MemoryFlash source; legacy_record(source,0,0,1); legacy_record(source,0,1,2);
  // Preparing an empty other bank needs exactly the three marker pages.
  cut_matrix(source,[](Journal& j){return j.initialize(GENERATION+1,0xCAFE);},
    [](Journal& j,MemoryFlash& f,unsigned,unsigned){
      if (j.state()==StorageState::legacy) CHECK(j.initialize(GENERATION+1,0xCAFE)==Status::ok);
      settle(j); CHECK(old_reader_rejects(f)); CHECK(j.allocate_candidate(1,false)==Status::ok);
      uint32_t identity; CHECK(j.identity(1,&identity,true) && identity==0xCB0001);
    });
  Journal j(source); CHECK(j.open()==StorageState::legacy && j.initialize(GENERATION+1,0xCAFE)==Status::ok);
  CHECK(j.maintain()==Status::ok); // fence
  // The still-present v1 bank must remain gated through a torn sector erase.
  cut_matrix(source,[](Journal& journal){return journal.maintain();},
    [](Journal& journal,MemoryFlash& f,unsigned,unsigned){
      CHECK(journal.state()==StorageState::initializing); settle(journal); CHECK(old_reader_rejects(f));
      CHECK(journal.allocate_candidate(1,false)==Status::ok); uint32_t id;
      CHECK(journal.identity(1,&id,true) && id!=0xCAFE01 && id!=0xCAFF01);
    },1);
}

int main() {
  lifecycle(); capacity_and_history(); bounds(); reset_v1(); initialization_cuts(); allocation_cuts();
  private_profile_policy(); primitive_bounds(); malformed_snapshots(); newer_bank_corruption(); full_bit_flip_sweep(); rotation_power_cuts(); legacy_reset_cuts();
  attempt_and_reservation_cuts(); replacement_and_retirement_cuts(); stop_budget(); corruption(); invalid_initializer_is_not_empty();
  printf("journal v2: OK (%u power cuts)\n",cut_checks);
}
