#include "journal.h"
#include <cassert>
#include <cstdio>
#include <cstring>

using namespace ha_x2d::journal;

int main() {
  MemoryFlash flash;
  FILE* fixture = fopen(JOURNAL_FIXTURE, "rb");
  assert(fixture);
  assert(fread(flash.raw(), 1, REGION_BYTES, fixture) == REGION_BYTES);
  assert(fgetc(fixture) == EOF && fclose(fixture) == 0);
  uint8_t before[REGION_BYTES];
  memcpy(before, flash.raw(), REGION_BYTES);
  Journal journal(flash);
  assert(journal.open() == StorageState::ready);
  assert(!memcmp(before, flash.raw(), REGION_BYTES));
  assert(flash.mutations() == 0);
  char generation[17];
  assert(journal.generation_hex(generation));
  assert(!strcmp(generation, "0123456789ABCDEF"));
  for (uint8_t slot = 1; slot <= SLOTS; ++slot) {
    const auto shutter = journal.shutter(slot);
    if (slot != 1 && slot != 7) {
      assert(shutter.state == SlotState::unused);
      continue;
    }
    uint32_t identity = 0, next = 0;
    assert(journal.identity(slot, &identity));
    assert(journal.next_counter(slot, &next));
    assert(identity == (slot == 1 ? 0x1234ABu : 0x9876CDu));
    assert(next == (slot == 1 ? 44u : 0xFFFEu));
    assert(shutter.state == (slot == 1 ? SlotState::paired : SlotState::pending));
    assert(shutter.last_command == (slot == 1 ? 3 : 0));
  }
  Reservation reservation;
  assert(journal.reserve(1, 3, true, &reservation) == Status::ok);
  assert(reservation.identity == 0x1234AB && reservation.counter == 44);
}
