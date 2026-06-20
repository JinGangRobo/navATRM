#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "custom_behaviors/nav_abort_tracker.hpp"

#include <behaviortree_cpp/condition_node.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <cstdint>
#include <memory>

namespace decision {
// Condition node that returns SUCCESS while a navigation abort is "fresh".
//
// It reads the shared NavAbortTracker (written by NavToPose on STATUS_ABORTED)
// and latches SUCCESS for `stuck_window_ms` after each abort, then returns
// FAILURE until the next abort. This gives the mission-layer recovery branch a
// bounded window in which to react to Nav2 giving up, without latching
// forever (which would trap the robot in recovery).
class IsNavStuck : public BT::ConditionNode {
public:
  IsNavStuck(const std::string &name, const BT::NodeConfig &config,
             std::shared_ptr<rclcpp::Clock> clock,
             std::shared_ptr<NavAbortTracker> tracker,
             const int64_t stuck_window_ms)
      : BT::ConditionNode(name, config), clock_(clock), tracker_(tracker),
        stuck_window_ns_(stuck_window_ms * 1000LL * 1000LL) {}

  BT::NodeStatus tick() override {
    const int64_t last = tracker_->last_abort_ns.load();
    if (last == 0) {
      return BT::NodeStatus::FAILURE;
    }
    const int64_t now = clock_->now().nanoseconds();
    if (now - last < stuck_window_ns_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts() { return {}; }

private:
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<NavAbortTracker> tracker_;
  int64_t stuck_window_ns_;
};
} // namespace decision
