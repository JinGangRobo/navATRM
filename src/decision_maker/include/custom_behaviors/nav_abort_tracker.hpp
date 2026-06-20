#pragma once

#include <atomic>
#include <cstdint>

namespace decision {
// Shared, lock-free state that records when the last navigation abort was
// observed. NavToPose writes to it whenever Nav2 returns STATUS_ABORTED;
// the IsNavStuck condition node reads it to decide whether a mission-layer
// recovery should run. Kept trivially copyable-friendly via a shared_ptr in
// the nodes that own it.
struct NavAbortTracker {
  // Nanoseconds since epoch of the last observed abort; 0 means "never".
  std::atomic<int64_t> last_abort_ns{0};
};
} // namespace decision
