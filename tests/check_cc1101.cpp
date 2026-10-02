// Native SPI/GPIO simulation: no RF or RP2040 headers.
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include <array>
#include <vector>
#include "cc1101.h"

using ha_x2d::cc1101::Driver;
using ha_x2d::cc1101::Mode;

struct Bus {
  std::array<uint8_t, 64> registers{};
  std::array<uint8_t, 2> pa{};
  std::vector<uint8_t> strobes;
  std::vector<std::vector<uint8_t>> transactions;
  std::vector<char> gpio;
  std::vector<bool> selections;
  uint32_t clock = 0, begins = 0, ends = 0;
  uint8_t state = 1, version = 4, corrupt_address = 0xFF;
  bool selected = false, active = false, stuck_miso = false;
  bool stuck_state = false, lose_spi_on_tx = false, lose_spi_on_reset = false;
  bool corrupt_pa = false, output = false, level = true;

  void begin_spi() { assert(!active); active = true; ++begins; transactions.emplace_back(); }
  void end_spi() { assert(active && !selected); active = false; ++ends; }
  void select(bool value) { selected = value; selections.push_back(value); }
  bool miso_high() { return stuck_miso; }
  uint32_t now_us() { clock += 10; return clock; }
  void delay_us(uint32_t us) { clock += us; }
  void data_write(bool value) { level = value; gpio.push_back(value ? 'H' : 'L'); }
  void data_output(bool value) {
    if (value && !output) {
      assert(!level);  // always low BEFORE output enable
      assert(registers[2] == 0x2E && registers[8] == 0x30);
    }
    output = value;
    gpio.push_back(value ? 'O' : 'I');
  }
  bool read_gpio(uint8_t pin) { return registers[pin == 20 ? 2 : 0] == 0x6F; }

  uint8_t transfer(uint8_t value) {
    assert(active && selected && !stuck_miso);
    auto& bytes = transactions.back();
    bytes.push_back(value);
    const uint8_t command = bytes.front();
    if (bytes.size() == 1) {
      if (command >= 0x30 && command <= 0x3D) {
        strobes.push_back(command);
        if (command == 0x30) {
          registers.fill(0);
          state = 1;
          if (lose_spi_on_reset) stuck_miso = true;
        } else if (!stuck_state) {
          if (command == 0x35) state = 0x13;
          else if (command == 0x34) state = 0x0D;
          else state = 1;
        }
        if (command == 0x35 && lose_spi_on_tx) stuck_miso = true;
      }
      return 0;
    }
    if (command == 0x7E) { pa.at(bytes.size() - 2) = value; return 0; }
    if (command == 0xFE) return pa.at(bytes.size() - 2) ^ (corrupt_pa ? 1 : 0);
    const uint8_t address = command & 0x3F;
    if (command & 0x80) {
      if (command & 0x40) {
        if (address == 0x30) return 0;
        if (address == 0x31) return version;
        if (address == 0x35) return state | 0x80;  // MARCSTATE must be masked
      }
      return registers[address] ^ (address == corrupt_address ? 1 : 0);
    }
    registers[address] = value;
    return 0;
  }
  void balanced() const { assert(!selected && !active && begins == ends); }
  void no_tx() const {
    assert(std::find(strobes.begin(), strobes.end(), 0x35) == strobes.end());
    for (const auto& bytes : transactions)
      if (!bytes.empty()) assert(bytes[0] != 0x7F && bytes[0] != 0x7E);
  }
};

int main() {
  {
    Bus bus;
    Driver<Bus> chip(bus);
    assert(chip.reset());
    assert(bus.selections.size() >= 5 && !bus.selections[0] && bus.selections[1] &&
           !bus.selections[2] && bus.selections[3] && !bus.selections[4]);
    const auto identity = chip.probe();
    assert(identity.detected && identity.partnum == 0 && identity.version == 4 && identity.marcstate == 1);
    bus.version = 0x14;
    assert(chip.probe().detected);
    bus.version = 0xFF;
    assert(!chip.probe().detected);
    assert(chip.write_register(0x0D, 0x21));
    uint8_t value = 0;
    assert(chip.read_register(0x0D, value) && value == 0x21);
    assert(bus.transactions.back()[0] == 0x8D);
    bus.corrupt_address = 0x0D;
    assert(!chip.write_register(0x0D, 0x42));
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus> chip(bus);
    bus.clock = UINT32_MAX - 1000;  // deadlines survive micros() rollover
    const uint32_t start = bus.clock;
    bus.stuck_miso = true;
    uint8_t value = 0xAA;
    assert(!chip.read_register(0, value) && value == 0xAA);
    assert(static_cast<uint32_t>(bus.clock - start) >= 2000 && chip.spi_errors() == 1);
    assert(bus.transactions.back().empty());
    bus.balanced();
    bus.stuck_miso = false;
    bus.lose_spi_on_reset = true;
    assert(!chip.reset());  // wait for MISO again AFTER SRES
    assert(bus.strobes.back() == 0x30 && chip.spi_errors() == 2);
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus> chip(bus);
    assert(!chip.begin_tx() && bus.gpio.empty());
    assert(chip.configure_transmitter() && chip.configured());
    assert(bus.pa[0] == 0 && bus.pa[1] == 0x50);
    // Original 26 MHz/868.35 MHz frequency, OOK, no packet engine, calibration.
    assert(bus.registers[0x0D] == 0x21 && bus.registers[0x0E] == 0x65 && bus.registers[0x0F] == 0xE8);
    assert(bus.registers[0x10] == 0x87 && bus.registers[0x11] == 0x85 && bus.registers[0x12] == 0x30);
    assert(bus.registers[0x16] == 0x11 && bus.registers[0x17] == 0 && bus.strobes.back() == 0x33);
    bus.gpio.clear();
    assert(chip.begin_tx() && chip.data_owned() && bus.state == 0x13);
    assert(bus.gpio.size() == 2 && bus.gpio[0] == 'L' && bus.gpio[1] == 'O');
    assert(!chip.release_data());
    assert(chip.idle() && bus.output);  // PIO abort must precede release
    assert(chip.release_data() && !bus.output && !chip.data_owned());
    bus.registers[0x02] = 0x0D;
    bus.gpio.clear();
    assert(!chip.begin_tx() && !chip.configured() && bus.gpio.empty());
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus> chip(bus);
    bus.corrupt_pa = true;
    assert(!chip.configure_transmitter() && !chip.configured());
    assert(std::find(bus.strobes.begin(), bus.strobes.end(), 0x33) == bus.strobes.end());
    assert(!chip.begin_tx() && !bus.output);
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus> chip(bus);
    assert(chip.configure_transmitter());
    bus.registers[8] = 0;  // packet mode cannot hand GDO0 to PIO
    bus.gpio.clear();
    assert(!chip.begin_tx() && !chip.configured() && bus.gpio.empty());
    assert(chip.configure_transmitter());
    bus.stuck_miso = true;
    bus.gpio.clear();
    assert(!chip.begin_tx() && !chip.configured() && bus.gpio.empty());
    assert(!chip.reset() && !chip.release_data());  // no stale IDLE permission
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus> chip(bus);
    assert(chip.configure_transmitter());
    bus.stuck_state = true;
    const uint32_t start = bus.clock;
    assert(!chip.begin_tx());
    assert(static_cast<uint32_t>(bus.clock - start) >= 20000);
    assert(!chip.data_owned() && !bus.output && chip.configured()); // confirmed IDLE cleanup
    bus.stuck_state = false;
    bus.lose_spi_on_tx = true;
    assert(!chip.begin_tx());
    assert(chip.data_owned() && bus.output && !bus.level && !chip.configured());
    assert(!chip.release_data() && !chip.idle() && bus.output && !bus.level);
    bus.balanced();
    bus.stuck_miso = false;
    assert(chip.idle() && chip.release_data() && !bus.output);
  }
  {
    Bus bus;
    Driver<Bus> chip(bus);
    assert(chip.configure_transmitter() && chip.begin_tx());
    chip.hold_data_low();  // digital DMA/PIO fault parks validated input
    assert(!bus.level && bus.output && !chip.configured());
    bus.stuck_miso = true;
    assert(!chip.idle() && !chip.release_data() && bus.output && !bus.level);
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus, Mode::rx_debug> chip(bus);
    assert(chip.reset() && chip.probe().detected);
    assert(chip.verify_wire(2, 20, 0x0D) && chip.verify_wire(0, 21, 0x0E));
    assert(chip.configure_receiver(false, 868350000, 0x43));
    assert(bus.state == 0x0D && bus.registers[2] == 0x0D && bus.registers[0] == 0x0E);
    assert(bus.registers[0x10] == 0x7A && bus.registers[0x11] == 0x93 && bus.registers[0x15] == 0x43);
    assert(bus.registers[0x16] == 7 && bus.registers[0x17] == 0x3C);
    assert(chip.configure_receiver(true, 433920000, 0xFF));
    assert(bus.registers[0x10] == 0x87 && bus.registers[0x15] == 0 && bus.registers[0x1D] == 0x92);
    assert(!chip.configure_transmitter() && !chip.begin_tx() && !chip.write_register(0x3F, 1));
    for (uint8_t strobe : bus.strobes) assert(strobe == 0x30 || strobe == 0x33 || strobe == 0x34 || strobe == 0x36);
    assert(bus.gpio.empty());
    bus.no_tx();
    bus.balanced();
  }
  {
    Bus bus;
    Driver<Bus, Mode::tx_check> chip(bus);
    ha_x2d::cc1101::DigitalInput input;
    assert(chip.configure_digital_check(input));
    assert(Driver<Bus>::digital_input_valid(input));
    assert(!chip.configure_receiver(true, 868350000, 0));
    assert(!chip.configure_transmitter() && !chip.begin_tx());
    assert(!chip.write_register(0x3F, 1) && !chip.write_register(0x3E, 0x50));
    bus.state = 0x13;
    assert(!chip.digital_input_verified(input));
    bus.stuck_state = true;
    const uint32_t start = bus.clock;
    assert(!chip.idle() && static_cast<uint32_t>(bus.clock - start) >= 20000);
    for (uint8_t strobe : bus.strobes) assert(strobe == 0x30 || strobe == 0x36);
    assert(bus.gpio.empty());
    bus.no_tx();
    bus.balanced();
  }
  puts("CC1101 simulated SPI/GPIO checks passed (no hardware/RF)");
}
