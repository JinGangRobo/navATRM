#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "custom_behaviors/nav_abort_tracker.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>

#include <chrono>
#include <limits>
#include <memory>

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;
namespace decision {
// Wraps the Nav2 NavigateToPose action as a BT stateful action.
//
// Robustness hardening over the original:
//   * Action server availability is retried a few times instead of failing on
//     the first cold-start miss.
//   * A resend cooldown rate-limits how often a goal is (re)issued, so a
//     ReactiveFallback that re-ticks this node cannot hammer the action server
//     at the BT tick rate after an abort.
//   * On STATUS_ABORTED (Nav2 truly gave up) an event is published on
//     "decision/nav_aborted" and recorded in the shared NavAbortTracker so the
//     mission layer (IsNavStuck) and external consumers (relocalization, UI)
//     can react.
//   * An `is_navigating` output port reflects whether a goal is currently in
//     flight, for use by motion-stall watchers.
class NavToPose : public StatefulActionNode {
public:
  NavToPose(const std::string &name, const NodeConfig &config,
            rclcpp_action::Client<NavigateToPose>::SharedPtr action_client,
            std::shared_ptr<rclcpp::Logger> logger,
            const int send_goal_timeout,
            rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nav_aborted_pub,
            std::shared_ptr<rclcpp::Clock> clock,
            std::shared_ptr<NavAbortTracker> abort_tracker,
            const int resend_cooldown_ms)
      : StatefulActionNode(name, config), action_client_(action_client),
        logger_(logger), send_goal_timeout_(send_goal_timeout),
        nav_aborted_pub_(nav_aborted_pub), clock_(clock),
        abort_tracker_(abort_tracker),
        resend_cooldown_(std::chrono::milliseconds(resend_cooldown_ms)) {
    RCLCPP_DEBUG(*logger_, "NAV2POSE NODE LOADED");
  }

  NodeStatus onStart() override {
    RCLCPP_DEBUG(*logger_, "NAV2POSE NODE ON START");
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
      RCLCPP_ERROR(*logger_, "goal is not set");
      return NodeStatus::FAILURE;
    }

    // Rate-limit (re)issues so a tight ReactiveFallback cannot spam the action
    // server after a failure. The first send is never blocked.
    if (has_sent_ &&
        (clock_->now() - last_send_time_).seconds() <
            std::chrono::duration<double>(resend_cooldown_).count()) {
      return NodeStatus::FAILURE;
    }

    navigation_goal_.pose = goal.value();
    navigation_goal_.pose.header.frame_id = "map";
    navigation_goal_.pose.header.stamp = clock_->now();

    // Retry server discovery: cold starts and transient DDS hiccups should not
    // surface as a hard navigation failure.
    constexpr int kServerWaitRetries = 3;
    constexpr auto kServerWaitStep = std::chrono::milliseconds(1500);
    bool server_available = false;
    for (int i = 0; i < kServerWaitRetries; ++i) {
      if (action_client_->wait_for_action_server(kServerWaitStep)) {
        server_available = true;
        break;
      }
    }
    if (!server_available) {
      RCLCPP_ERROR(*logger_, "Action server not available");
      return NodeStatus::FAILURE;
    }

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto future_goal_handle =
        action_client_->async_send_goal(navigation_goal_, send_goal_options);

    RCLCPP_DEBUG(*logger_, "send goal timeout ms: %d", send_goal_timeout_);

    // 使用 wait_for 替代 spin_until_future_complete
    if (future_goal_handle.wait_for(std::chrono::milliseconds(
            send_goal_timeout_)) != std::future_status::ready) {
      RCLCPP_ERROR(*logger_, "send goal failed");
      return NodeStatus::FAILURE;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(*logger_, "goal handle is null");
      return NodeStatus::FAILURE;
    }

    has_sent_ = true;
    last_send_time_ = clock_->now();
    setOutput<bool>("is_navigating", true);

    RCLCPP_INFO(*logger_, "Navigating to pose [%f, %f, %f]",
                goal->pose.position.x, goal->pose.position.y,
                goal->pose.position.z);
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override {
    auto new_goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!new_goal) {
      RCLCPP_ERROR(*logger_, "goal is not set");
      return NodeStatus::FAILURE;
    }

    if ((new_goal.value().pose.position.x -
         navigation_goal_.pose.pose.position.x) >
            std::numeric_limits<double>::epsilon() ||
        (new_goal.value().pose.position.y -
         navigation_goal_.pose.pose.position.y) >
            std::numeric_limits<double>::epsilon() ||
        (new_goal.value().pose.position.z -
         navigation_goal_.pose.pose.position.z) >
            std::numeric_limits<double>::epsilon()) {
      RCLCPP_INFO(*logger_, "goal updated");

      navigation_goal_.pose = new_goal.value();
      navigation_goal_.pose.header.frame_id = "map";
      navigation_goal_.pose.header.stamp = clock_->now();

      auto send_goal_options =
          rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      auto future_goal_handle =
          action_client_->async_send_goal(navigation_goal_, send_goal_options);

      if (future_goal_handle.wait_for(std::chrono::milliseconds(
              send_goal_timeout_)) != std::future_status::ready) {
        RCLCPP_ERROR(*logger_, "send goal failed");
        return NodeStatus::FAILURE;
      }

      goal_handle_ = future_goal_handle.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(*logger_, "goal handle is null");
        return NodeStatus::FAILURE;
      }
      last_send_time_ = clock_->now();
      RCLCPP_INFO(*logger_, "Navigating to pose [%f, %f, %f]",
                  new_goal->pose.position.x, new_goal->pose.position.y,
                  new_goal->pose.position.z);
    }

    switch (goal_handle_->get_status()) {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      RCLCPP_DEBUG(*logger_, "goal status: STATUS_UNKNOWN");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      RCLCPP_DEBUG(*logger_, "goal status: STATUS_ACCEPTED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      RCLCPP_DEBUG(*logger_, "goal status: STATUS_EXECUTING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      RCLCPP_INFO(*logger_, "goal status: STATUS_CANCELING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(*logger_, "goal status: STATUS_SUCCEEDED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_INFO(*logger_, "goal status: STATUS_CANCELED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_INFO(*logger_, "goal status: STATUS_ABORTED");
      break;
    default:
      RCLCPP_INFO(*logger_, "goal status: ERROR CODE");
      break;
    }

    if (goal_handle_->get_status() ==
        action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      setOutput<bool>("is_navigating", false);
      return NodeStatus::SUCCESS;
    } else if (goal_handle_->get_status() ==
                   action_msgs::msg::GoalStatus::STATUS_ABORTED ||
               goal_handle_->get_status() ==
                   action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      // Nav2 gave up (or was canceled): surface an abort event so the mission
      // layer and external consumers (relocalization, UI) can react.
      if (goal_handle_->get_status() ==
          action_msgs::msg::GoalStatus::STATUS_ABORTED) {
        signalAbort();
      }
      setOutput<bool>("is_navigating", false);
      return NodeStatus::FAILURE;
    } else {
      return NodeStatus::RUNNING;
    }
  }

  void onHalted() override {
    RCLCPP_INFO(*logger_, "goal halted");
    if (goal_handle_) {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (cancel_future.wait_for(std::chrono::seconds(1)) !=
          std::future_status::ready) {
        RCLCPP_ERROR(*logger_, "cancel goal failed");
      }
      RCLCPP_INFO(*logger_, "goal canceled");
    }
    setOutput<bool>("is_navigating", false);
  }

  static PortsList providedPorts() {
    const char *description = "goal send to navigator.";
    return {InputPort<geometry_msgs::msg::PoseStamped>("goal", description),
            OutputPort<bool>("is_navigating",
                             "true while a navigation goal is in flight")};
  }

private:
  // Publish the abort event and record it in the shared tracker.
  void signalAbort() {
    if (nav_aborted_pub_) {
      nav_aborted_pub_->publish(std_msgs::msg::Empty());
    }
    if (abort_tracker_) {
      abort_tracker_->last_abort_ns.store(clock_->now().nanoseconds());
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  NavigateToPoseGoalHandle::SharedPtr goal_handle_;
  std::shared_ptr<rclcpp::Logger> logger_;

  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  int send_goal_timeout_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nav_aborted_pub_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<NavAbortTracker> abort_tracker_;

  std::chrono::milliseconds resend_cooldown_;
  rclcpp::Time last_send_time_;
  bool has_sent_{false};
};
} // namespace decision
