#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "custom_behaviors/odom_tracker.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cmath>
#include <cstdint>
#include <memory>

namespace decision {
// Condition node that reports a physical motion stall: the robot is nominally
// navigating (the NavToPose `is_navigating` blackboard flag is true) yet its
// linear speed has stayed below `stall_threshold` for longer than
// `stall_duration_ms`.
//
// This targets the gap where Nav2 has NOT aborted (e.g. it is stuck looping in
// ComputePath, or the controller is happily emitting near-zero commands) so the
// progress checker / abort path never fires. It only acts while a navigation
// goal is actually in flight, which avoids false positives when the robot is
// legitimately stopped at a goal or idling.
//
// NOTE: wiring this into a mission tree should be done with care — a
// motion-stall response at the mission layer (e.g. a retreat) is more
// aggressive than letting Nav2's own Spin/Wait recovery handle it, so the
// thresholds need on-robot tuning before it is enabled.
class IsMotionStalled : public BT::ConditionNode {
public:
  IsMotionStalled(const std::string &name, const BT::NodeConfig &config,
                  std::shared_ptr<rclcpp::Clock> clock,
                  std::shared_ptr<OdomTracker> odom_tracker,
                  const double stall_threshold,
                  const int64_t stall_duration_ms)
      : BT::ConditionNode(name, config), clock_(clock),
        odom_tracker_(odom_tracker),
        threshold_sq_(stall_threshold * stall_threshold),
        stall_duration_ns_(stall_duration_ms * 1000LL * 1000LL) {}

  BT::NodeStatus tick() override {
    const bool is_navigating = getInput<bool>("is_navigating").value_or(false);
    if (!is_navigating) {
      stall_start_ns_ = 0;
      return BT::NodeStatus::FAILURE;
    }

    const int64_t now = clock_->now().nanoseconds();
    const int64_t last_odom = odom_tracker_->last_update_ns.load();
    // No odom, or odom went stale: cannot determine stall, stay safe.
    if (last_odom == 0 || (now - last_odom) > kOdomStaleNs) {
      return BT::NodeStatus::FAILURE;
    }

    const double speed_sq = odom_tracker_->speed_sq.load();
    if (speed_sq >= threshold_sq_) {
      stall_start_ns_ = 0;
      return BT::NodeStatus::FAILURE;
    }

    if (stall_start_ns_ == 0) {
      stall_start_ns_ = now;
    }
    if (now - stall_start_ns_ >= stall_duration_ns_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>(
        "is_navigating",
        "true while a NavToPose goal is in flight (written by NavToPose)")};
  }

private:
  static constexpr int64_t kOdomStaleNs = 1000LL * 1000LL * 1000LL; // 1 s

  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<OdomTracker> odom_tracker_;
  double threshold_sq_;
  int64_t stall_duration_ns_;
  int64_t stall_start_ns_{0};
};
} // namespace decision
