#pragma once

#include <atomic>
#include <cstdint>

namespace decision {
// Shared, lock-free record of the latest linear speed observed on the odom
// topic. DecisionMaker owns the odom subscription and writes here; the
// IsMotionStalled condition node reads here. We store speed squared to avoid a
// sqrt on every odom sample.
struct OdomTracker {
  void update(const double vx, const double vy, const int64_t now_ns) {
    speed_sq.store(vx * vx + vy * vy, std::memory_order_relaxed);
    last_update_ns.store(now_ns, std::memory_order_relaxed);
  }

  std::atomic<double> speed_sq{0.0};
  std::atomic<int64_t> last_update_ns{0}; // 0 = no odom yet
};
} // namespace decision
