#pragma once

#include <stdint.h>

namespace ha_x2d::cc1101 {

enum class Mode { gateway, rx_debug, tx_check };
struct Identity {
  bool detected = false;
  uint8_t partnum = 0, version = 0, marcstate = 0;
};
struct DigitalInput {
  uint8_t marcstate = 0, iocfg0 = 0, pktctrl0 = 0;
};

// Bus supplies elementary SPI, CS/MISO, GPIO and microsecond operations only.
// PIO/DMA ownership stays with the firmware; release_data follows its abort.
template <class Bus, Mode mode = Mode::gateway>
class Driver {
 public:
  explicit Driver(Bus& bus) : bus_(bus) {}

  uint32_t spi_errors() const { return spi_errors_; }
  bool configured() const { return configured_; }
  bool data_owned() const { return data_owned_; }

  bool read_register(uint8_t address, uint8_t& value, bool status = false) {
    if ((status ? (address < 0x30 || address > 0x3D) : address > 0x2E) ||
        !begin()) return false;
    bus_.transfer(address | (status ? 0xC0 : 0x80));
    value = bus_.transfer(0);
    end();
    return true;
  }

  bool write_register(uint8_t address, uint8_t value) {
    // Never expose FIFO/PATABLE writes to diagnostics.
    if (address > 0x2E || !begin()) return false;
    bus_.transfer(address);
    bus_.transfer(value);
    end();
    uint8_t observed;
    return read_register(address, observed) && observed == value;
  }

  bool wait_state(uint8_t expected) {
    const uint32_t start = bus_.now_us();
    do {
      uint8_t state;
      if (!read_register(0x35, state, true)) return false;
      if ((state & 0x1F) == expected) return true;
      if constexpr (mode != Mode::tx_check)
        bus_.delay_us(mode == Mode::rx_debug ? 100 : 50);
    } while (static_cast<uint32_t>(bus_.now_us() - start) < 20000);
    return false;
  }

  bool reset() {
    configured_ = false;
    idle_verified_ = false;
    bus_.select(false); bus_.delay_us(40);
    bus_.select(true); bus_.delay_us(10);
    bus_.select(false); bus_.delay_us(40);
    if (!begin()) return false;
    bus_.transfer(0x30);
    const bool ready = wait_ready();
    end();
    tx_active_ = false;
    idle_verified_ = ready && wait_state(1);
    return idle_verified_;
  }

  Identity probe() {
    Identity result;
    if (!read_register(0x30, result.partnum, true) ||
        !read_register(0x31, result.version, true) ||
        !read_register(0x35, result.marcstate, true)) return result;
    result.marcstate &= 0x1F;
    result.detected = result.partnum == 0 &&
                      (result.version == 4 || result.version == 0x14);
    return result;
  }

  bool idle() {
    idle_verified_ = strobe(0x36) && wait_state(1);
    tx_active_ = false;
    if (!idle_verified_) hold_data_low();
    return idle_verified_;
  }

  bool read_digital_input(DigitalInput& result) {
    const bool ok = read_register(0x35, result.marcstate, true) &&
                    read_register(0x02, result.iocfg0) &&
                    read_register(0x08, result.pktctrl0);
    result.marcstate &= 0x1F;
    return ok;
  }

  bool digital_input_verified(DigitalInput& result) {
    return read_digital_input(result) && digital_input_valid(result);
  }

  static bool digital_input_valid(const DigitalInput& result) {
    return result.marcstate == 1 && result.iocfg0 == 0x2E && result.pktctrl0 == 0x30;
  }

  bool configure_digital_check(DigitalInput& result) {
    if constexpr (mode != Mode::tx_check) return false;
    return reset() && idle() && probe().detected &&
           write_register(0x02, 0x2E) && write_register(0x08, 0x30) &&
           idle() && digital_input_verified(result);
  }

  bool configure_receiver(bool ook, uint32_t frequency_hz, uint8_t deviation) {
    if constexpr (mode != Mode::rx_debug) return false;
    if (!idle()) return false;
    const uint32_t freq = frequency_word(frequency_hz);
    const uint8_t registers[][2] = {
        {0x00, 0x0E}, {0x01, 0x2E}, {0x02, 0x0D}, {0x03, 0x47},
        {0x07, 0x00}, {0x08, 0x30}, {0x0A, 0x00}, {0x0B, 0x06}, {0x0C, 0x00},
        {0x0D, static_cast<uint8_t>(freq >> 16)},
        {0x0E, static_cast<uint8_t>(freq >> 8)}, {0x0F, static_cast<uint8_t>(freq)},
        {0x13, 0x00}, {0x16, 0x07}, {0x17, 0x3C}, {0x18, 0x08},
        {0x21, 0xB6}, {0x2C, 0x81}, {0x2D, 0x35},
        {0x10, static_cast<uint8_t>(ook ? 0x87 : 0x7A)},
        {0x11, static_cast<uint8_t>(ook ? 0x85 : 0x93)},
        {0x12, static_cast<uint8_t>(ook ? 0x30 : 0x00)},
        {0x15, static_cast<uint8_t>(ook ? 0 : deviation)},
        {0x19, static_cast<uint8_t>(ook ? 0x14 : 0x16)},
        {0x1B, static_cast<uint8_t>(ook ? 0x04 : 0x03)},
        {0x1C, static_cast<uint8_t>(ook ? 0x00 : 0x40)},
        {0x1D, static_cast<uint8_t>(ook ? 0x92 : 0x91)}};
    for (const auto& reg : registers)
      if (!write_register(reg[0], reg[1])) return false;
    if (strobe(0x33) && wait_state(1) && strobe(0x34) && wait_state(0x0D)) return true;
    strobe(0x36);
    return false;
  }

  bool verify_wire(uint8_t address, uint8_t pin, uint8_t restore) {
    if constexpr (mode != Mode::rx_debug) return false;
    const bool low_written = write_register(address, 0x2F);
    bus_.delay_us(50);
    const bool low = !bus_.read_gpio(pin);
    const bool high_written = write_register(address, 0x6F);
    bus_.delay_us(50);
    const bool high = bus_.read_gpio(pin);
    const bool restored = write_register(address, restore);
    return low_written && high_written && low && high && restored;
  }

  bool configure_transmitter() {
    if constexpr (mode != Mode::gateway) return false;
    configured_ = false;
    if (data_owned_ && !idle()) return false;
    bus_.data_output(false);
    data_owned_ = false;
    if (!idle()) return false;
    constexpr uint32_t freq = frequency_word(868350000);
    const uint8_t registers[][2] = {
        {0x00, 0x2E}, {0x01, 0x2E}, {0x02, 0x2E},
        {0x07, 0}, {0x08, 0x30}, {0x0A, 0}, {0x0B, 6}, {0x0C, 0},
        {0x0D, static_cast<uint8_t>(freq >> 16)},
        {0x0E, static_cast<uint8_t>(freq >> 8)}, {0x0F, static_cast<uint8_t>(freq)},
        {0x10, 0x87}, {0x11, 0x85}, {0x12, 0x30}, {0x13, 0}, {0x15, 0},
        {0x16, 0x11}, {0x17, 0x00}, {0x18, 0x08},
        {0x21, 0xB6}, {0x22, 0x11}, {0x2C, 0x81}, {0x2D, 0x35}};
    for (const auto& reg : registers)
      if (!write_register(reg[0], reg[1])) return false;
    if (!begin()) return false;
    bus_.transfer(0x7E); bus_.transfer(0); bus_.transfer(0x50);
    end();
    if (!begin()) return false;
    bus_.transfer(0xFE);
    const uint8_t off = bus_.transfer(0), on = bus_.transfer(0);
    end();
    configured_ = off == 0 && on == 0x50 && strobe(0x33) && wait_state(1);
    return configured_;
  }

  bool begin_tx() {
    if constexpr (mode != Mode::gateway) return false;
    if (!configured_) return false;
    uint8_t iocfg0, pktctrl0;
    if (!read_register(0x02, iocfg0) || iocfg0 != 0x2E ||
        !read_register(0x08, pktctrl0) || pktctrl0 != 0x30) {
      hold_data_low();
      return false;
    }
    if (tx_active_) return true;
    bus_.data_write(false);
    bus_.data_output(true);
    data_owned_ = true;
    idle_verified_ = false;
    if (strobe(0x35) && wait_state(0x13)) {
      tx_active_ = true;
      return true;
    }
    if (idle()) release_data();
    return false;
  }

  void hold_data_low() {
    configured_ = false;
    if (!data_owned_) return;  // never drive an unverified CC1101 output
    bus_.data_write(false);
    bus_.data_output(true);
  }

  bool release_data() {
    if (!idle_verified_) return false;
    if (data_owned_) bus_.data_output(false);
    data_owned_ = false;
    return true;
  }

  static constexpr uint32_t frequency_word(uint32_t hz) {
    return (uint64_t{hz} * 65536 + 13000000) / 26000000;
  }

 private:
  bool wait_ready() {
    const uint32_t start = bus_.now_us();
    while (bus_.miso_high()) {
      if (static_cast<uint32_t>(bus_.now_us() - start) >= 2000) {
        ++spi_errors_;
        return false;
      }
    }
    return true;
  }
  bool begin() {
    bus_.begin_spi();
    bus_.select(true);
    if (wait_ready()) return true;
    end();
    return false;
  }
  void end() { bus_.select(false); bus_.end_spi(); }
  bool strobe(uint8_t command) {
    const bool allowed = command == 0x36 ||
        (mode != Mode::tx_check && command == 0x33) ||
        (mode == Mode::rx_debug && command == 0x34) ||
        (mode == Mode::gateway && command == 0x35 && data_owned_);
    if (!allowed || !begin()) return false;
    bus_.transfer(command);
    end();
    return true;
  }

  Bus& bus_;
  uint32_t spi_errors_ = 0;
  bool configured_ = false, data_owned_ = false, tx_active_ = false, idle_verified_ = false;
};

}  // namespace ha_x2d::cc1101
