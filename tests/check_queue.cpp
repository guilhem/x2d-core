#include <assert.h>
#include <string>
#include <vector>
#include <x2d/tx_queue.h>
using namespace x2d;
int main() {
  TxQueue queue;
  std::vector<std::string> outcomes;
  auto report = [&](const TxJob&, const char* value) { outcomes.emplace_back(value); };
  assert(queue.push({1,0,1,Action::open,false,1},100,report));
  assert(queue.push({2,0,2,Action::close,false,1},100,report));
  assert(queue.push({3,0,1,Action::stop,false,1},101,report));
  assert(outcomes.size() == 1 && outcomes[0] == "cancelled");
  TxJob job;
  assert(queue.pop(101,job,report) && job.request_id == 3);
  assert(queue.pop(101,job,report) && job.request_id == 2);
  assert(!queue.pop(101,job,report));
  assert(queue.push({4,0,1,Action::open,false,1},0xfffffff0u,report));
  assert(!queue.pop(0xfffffff0u + TxQueue::TTL_MS,job,report));
  assert(outcomes.back() == "expired");
  for (size_t i = 0; i < TxQueue::CAPACITY; ++i)
    assert(queue.push({static_cast<uint32_t>(i+5),0,2,Action::close,false,1},10,report));
  assert(!queue.push({30,0,3,Action::open,false,1},10,report));
  assert(queue.push({31,0,3,Action::stop,false,1},10,report));
  assert(queue.pop(10,job,report) && job.request_id == 31);
  queue.clear(report); assert(!queue.size());
  assert(queue.push({60,0,1,Action::open,false,1},10,report));
  assert(queue.push({61,0,2,Action::close,false,1},10,report));
  assert(queue.push({62,0,1,Action::none,true,1},10,report));
  std::vector<uint32_t> cancelled;
  queue.cancel_slot(1, [&](const TxJob& removed, const char* outcome) {
    assert(std::string(outcome) == "cancelled");
    cancelled.push_back(removed.request_id);
  });
  assert((cancelled == std::vector<uint32_t>{60,62}));
  assert(queue.pop(10,job,report) && job.request_id == 61);
  assert(!queue.size());
  // A STOP for a new incarnation cannot coalesce a previous incarnation's job.
  assert(queue.push({70,0,1,Action::open,false,1},10,report));
  assert(queue.push({71,0,1,Action::stop,false,2},10,report));
  assert(queue.size() == 2);
  assert(queue.pop(10,job,report) && job.request_id == 71 && job.incarnation == 2);
  assert(queue.pop(10,job,report) && job.request_id == 70 && job.incarnation == 1);
  for (uint8_t i = 1; i <= MAX_SHUTTERS; ++i)
    assert(queue.push({static_cast<uint32_t>(31+i),0,i,Action::stop,false,1},10,report));
  assert(queue.push({50,0,1,Action::stop,false,1},10,report));
  assert(queue.size() == TxQueue::CAPACITY);
}
