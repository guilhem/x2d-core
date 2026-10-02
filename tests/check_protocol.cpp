#include <assert.h>
#include <string>
#include <vector>
#include "protocol.h"
#include "tx_queue.h"
using namespace ha_x2d;
Request parse(const std::string& s) { return decode(s.data(), s.size()); }
int main() {
  OutputBuffer output;
  const std::string fill(OutputBuffer::CAPACITY - 3, 'x');
  assert(output.append(fill.data(), fill.size()));
  assert(!output.append("abcd", 4));
  assert(output.size() == fill.size());
  output.consume(OutputBuffer::CAPACITY - 5);
  assert(output.append("hello\n", 6));
  std::string drained;
  while (output.size()) {
    const size_t count = output.contiguous() < 2 ? output.contiguous() : 2;
    drained.append(output.data(), count);
    output.consume(count);
  }
  assert(drained == "xxhello\n");
  assert(output.append("old\n", 4));
  output.clear();
  assert(output.size() == 0 && !output.append(nullptr, 1));
  assert(parse(R"({"v":2,"id":1,"op":"hello"})").op == Operation::hello);
  const std::string prefix = R"({"v":2,"id":2,"session":"0123456789ABCDEF","op":)";
  assert(parse(prefix + R"("command","args":{"shutter_id":1,"action":"stop"}})").action == Action::stop);
  for (const std::string& s : std::vector<std::string>{
      R"({"v":1,"id":1,"op":"hello"})", R"({"v":true,"id":1,"op":"hello"})",
      R"({"v":2,"id":true,"op":"hello"})", R"({"v":2,"id":1,"op":"hello","op":"hello"})",
      R"({'v':2,'id':1,'op':'hello'})", R"({v:2,id:1,op:"hello"})",
      R"({"v":2,"id":01,"op":"hello"})", R"({"v":2,"id":1,"op":"hello"} {})",
      prefix + R"("command","args":{"shutter_id":1,"shutter_id":2,"action":"stop"}})",
      prefix + R"("command","args":{"shutter_id":0,"action":"stop"}})",
      prefix + R"("command","args":{"shutter_id":17,"action":"stop"}})",
      prefix + R"("command","args":{"shutter_id":1,"action":"pair"}})",
      prefix + R"("status","args":{"x":1}})"}) assert(parse(s).error);
  LineFramer framer;
  for (size_t i = 0; i < MAX_LINE_BYTES; ++i) assert(framer.feed('x') == LineFramer::Event::none);
  assert(framer.feed('\n') == LineFramer::Event::too_long);
  for (char c : std::string("hello\r")) framer.feed(c);
  assert(framer.feed('\n') == LineFramer::Event::line && framer.length == 5);
  TxQueue queue;
  std::vector<std::string> outcomes;
  auto report = [&](const TxJob&, const char* value) { outcomes.emplace_back(value); };
  assert(queue.push({1,0,1,Action::open,false},100,report));
  assert(queue.push({2,0,2,Action::close,false},100,report));
  assert(queue.push({3,0,1,Action::stop,false},101,report));
  assert(outcomes.size() == 1 && outcomes[0] == "cancelled");
  TxJob job;
  assert(queue.pop(101,job,report) && job.request_id == 3);
  assert(queue.pop(101,job,report) && job.request_id == 2);
  assert(!queue.pop(101,job,report));
  assert(queue.push({4,0,1,Action::open,false},0xfffffff0u,report));
  assert(!queue.pop(0xfffffff0u + TxQueue::TTL_MS,job,report));
  assert(outcomes.back() == "expired");
  for (size_t i = 0; i < TxQueue::CAPACITY; ++i)
    assert(queue.push({static_cast<uint32_t>(i+5),0,2,Action::close,false},10,report));
  assert(!queue.push({30,0,3,Action::open,false},10,report));
  assert(queue.push({31,0,3,Action::stop,false},10,report));
  assert(queue.pop(10,job,report) && job.request_id == 31);
  queue.clear(report); assert(!queue.size());
  for (uint8_t i = 1; i <= MAX_SHUTTERS; ++i)
    assert(queue.push({static_cast<uint32_t>(31+i),0,i,Action::stop,false},10,report));
  assert(queue.push({50,0,1,Action::stop,false},10,report));
  assert(queue.size() == TxQueue::CAPACITY);
}
