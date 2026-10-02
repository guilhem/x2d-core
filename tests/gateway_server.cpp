// Host gateway for the Python PTY/TCP bridges: the real shared Gateway (gateway.h)
// over simulated flash and radio, with stdin/stdout as the byte transport. The
// flash survives connections; only the bridge decides when a connection ends.
//
//   gateway_server [--control-fd=N] [--burst-ms=N] [--device-id=HEX16] [--session=HEX16]
//
// stdin/stdout carry protocol bytes only. EOF on stdin exits after the output
// is flushed. --control-fd is an inherited, bidirectional stream socket:
//   bridge -> server  'D'  link lost: gateway.disconnected(), buffered input and
//                          unread stdin discarded, then 'D' is echoed as the ack.
//                          Every stdout byte written earlier is already in the
//                          pipe by then, so the bridge drains and discards stdout
//                          up to the ack and may then reconnect.
//                    'H'  hold the radio: burst progress freezes; echoed.
//                    'R'  release the radio; echoed.
//   server -> bridge  'F'  gateway.failed() (output overflow): close the link,
//                          then send 'D'. Repeats once per connection.
// The simulation: shutters 1 and 2 paired, commands enabled, enrollment off;
// a full 25-copy burst lasts --burst-ms (default 250). The clock is monotonic
// real time shared by the gateway and the radio. Without --control-fd, the link
// can only end with EOF.
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <string>

#include "sim_gateway.h"

using namespace ha_x2d;

namespace {

struct Options {
  int control = -1;
  uint32_t burst_ms = 250;
  std::string device_id = sim::DEVICE_ID, session = sim::SESSION;
};

bool number(const std::string& text, uint32_t maximum, uint32_t& out) {
  char* end = nullptr;
  errno = 0;
  const unsigned long value = strtoul(text.c_str(), &end, 10);
  if (text.empty() || *end || errno || value > maximum) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

bool parse(int argc, char** argv, Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const size_t eq = arg.find('=');
    const std::string name = arg.substr(0, eq);
    std::string value;
    if (eq != std::string::npos) value = arg.substr(eq + 1);
    else if (i + 1 < argc) value = argv[++i];
    uint32_t parsed = 0;
    if (name == "--control-fd" && number(value, 1023, parsed)) options.control = static_cast<int>(parsed);
    else if (name == "--burst-ms" && number(value, 600000, parsed) && parsed) options.burst_ms = parsed;
    else if (name == "--device-id") options.device_id = value;
    else if (name == "--session") options.session = value;
    else return false;
  }
  return true;
}

void reply(int control, char byte) {
  if (control >= 0 && write(control, &byte, 1) < 0) {}  // the bridge going away shows as EOF
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse(argc, argv, options)) {
    fputs("usage: gateway_server [--control-fd=N] [--burst-ms=N] [--device-id=HEX16] [--session=HEX16]\n",
          stderr);
    return 2;
  }
  signal(SIGPIPE, SIG_IGN);
  for (int fd : {1, options.control})
    if (fd >= 0) fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);

  const auto epoch = std::chrono::steady_clock::now();
  uint32_t clock = 0;
  auto update_clock = [&] {
    clock = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::steady_clock::now() - epoch).count());
  };

  journal::MemoryFlash flash;
  journal::Journal journal(flash);
  sim::open_paired(journal);
  sim::Radio radio(clock);
  radio.copy_ms = options.burst_ms >= 25 ? options.burst_ms / 25 : 1;
  sim::Hooks hooks(clock);
  sim::Gateway gateway(journal, radio, hooks, options.device_id.c_str(), options.session.c_str());
  if (gateway.failed()) {
    fputs("--device-id and --session must be uppercase hex16\n", stderr);
    return 2;
  }
  gateway.begin();

  char rx[256];  // like the firmware: a small window, one line per feed
  size_t rx_used = 0, rx_pos = 0;
  bool eof = false, failure_reported = false;

  auto flush = [&] {
    while (gateway.output_size()) {
      const ssize_t put = write(1, gateway.output_data(), gateway.output_contiguous());
      if (put > 0) gateway.consume_output(static_cast<size_t>(put));
      else if (errno == EINTR) continue;
      else if (errno == EAGAIN) return true;
      else return false;  // the reader is gone
    }
    return true;
  };

  for (;;) {
    pollfd fds[2];
    nfds_t count = 0;
    int in_slot = -1, control_slot = -1;
    if (!eof && !gateway.failed() && rx_pos == rx_used) {
      in_slot = static_cast<int>(count);
      fds[count++] = {0, POLLIN, 0};
    }
    if (options.control >= 0) {
      control_slot = static_cast<int>(count);
      fds[count++] = {options.control, POLLIN, 0};
    }
    poll(fds, count, rx_pos < rx_used ? 0 : 1);

    if (control_slot >= 0 && (fds[control_slot].revents & (POLLIN | POLLHUP))) {
      char byte;
      const ssize_t got = read(options.control, &byte, 1);
      if (got == 0) return 0;  // bridge closed the control channel
      if (got == 1 && byte == 'D') {
        update_clock();
        gateway.disconnected();
        rx_used = rx_pos = 0;
        failure_reported = false;
        for (;;) {  // bytes of the lost connection still unread on stdin
          pollfd stale{0, POLLIN, 0};
          char junk[4096];
          if (poll(&stale, 1, 0) <= 0 || !(stale.revents & (POLLIN | POLLHUP))) break;
          const ssize_t dropped = read(0, junk, sizeof(junk));
          if (dropped <= 0) {
            eof = eof || dropped == 0;
            break;
          }
        }
        reply(options.control, 'D');
      } else if (got == 1 && (byte == 'H' || byte == 'R')) {
        radio.held = byte == 'H';
        reply(options.control, byte);
      } else if (got == 1) reply(options.control, '?');
    }
    if (in_slot >= 0 && (fds[in_slot].revents & (POLLIN | POLLHUP))) {
      const ssize_t got = read(0, rx, sizeof(rx));
      if (got > 0) {
        rx_used = static_cast<size_t>(got);
        rx_pos = 0;
      } else if (got == 0 || (errno != EINTR && errno != EAGAIN)) eof = true;
    }

    update_clock();
    if (rx_pos < rx_used) rx_pos += gateway.feed(rx + rx_pos, rx_used - rx_pos);
    gateway.tick();  // after feed(): the reply is queued before any completion event
    if (!flush()) return 1;
    if (gateway.failed() && !failure_reported) {
      failure_reported = true;
      reply(options.control, 'F');
    }
    if (eof && rx_pos == rx_used && !gateway.output_size()) return 0;
  }
}
