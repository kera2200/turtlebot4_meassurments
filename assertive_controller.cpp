#include "assertive_controller/assertive_controller.hpp"

namespace assertive_controller
{

void AssertiveController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  RCLCPP_INFO(node_->get_logger(), "AssertiveController has been configured.");
}

void AssertiveController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up AssertiveController");
}

void AssertiveController::activate()
{
  RCLCPP_INFO(logger_, "Activating AssertiveController");
}

void AssertiveController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating AssertiveController");
}

void AssertiveController::setPlan(const nav_msgs::msg::Path & path)
{
   

    if (path.poses.empty()) {
        RCLCPP_WARN(logger_, "Received an empty path. Ignoring.");
        return;
    }

    RCLCPP_INFO(logger_, "Setting path in AssertiveController with %lu poses", path.poses.size());
    plan_ = path;
    const auto & start = path.poses.front().pose.position;
    const auto & goal = path.poses.back().pose.position;
    movement_angle_ = std::atan2(goal.y - start.y, goal.x - start.x);
    RCLCPP_INFO(logger_, "Fixed movement angle: %.2f radians", movement_angle_);
}

geometry_msgs::msg::TwistStamped AssertiveController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  RCLCPP_INFO(node_->get_logger(), "[AssertiveController] computeVelocityCommands() called");

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();

  // Prune passed waypoints
  while (plan_.poses.size() > 1) {
    const auto & target_pose = plan_.poses.front().pose;
    double dx = target_pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.position.y - current_pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist > 0.01) break;
    RCLCPP_INFO(node_->get_logger(), "Passed a waypoint (dist=%.2f), removing it", dist);
    plan_.poses.erase(plan_.poses.begin());
  }

  if (plan_.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Empty path received");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

 

  double current_angle = tf2::getYaw(current_pose.pose.orientation);
  // Obstacle check
  unsigned int mx, my;
  double check_x = current_pose.pose.position.x + std::cos(current_angle) * 0.4;
  double check_y = current_pose.pose.position.y + std::sin(current_angle) * 0.4;


  RCLCPP_INFO(node_->get_logger(), "Checking costmap at (x=%.2f, y=%.2f)", check_x, check_y);

  if (costmap_ros_->getCostmap()->worldToMap(check_x, check_y, mx, my)) {
      unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
      RCLCPP_INFO(node_->get_logger(), "Costmap value at (mx=%u, my=%u): %u", mx, my, cost);
      if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          RCLCPP_INFO(node_->get_logger(), "Obstacle detected. Stopping robot.");
          cmd_vel.twist.linear.x = 0.0;
          cmd_vel.twist.angular.z = 0.0;
          return cmd_vel;  // Stop the robot
      }
      
  } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to query costmap at (x=%.2f, y=%.2f)", check_x, check_y);
      //obstacle_detected_ = false;  // Reset obstacle detection if costmap query fails
  }


  // Angle calculations
  double angle_diff = std::atan2(std::sin(movement_angle_ - current_angle),
                                 std::cos(movement_angle_ - current_angle));

  RCLCPP_INFO(node_->get_logger(), "movement_angle: %.4f, current_angle: %.4f, angle_diff: %.4f",
              movement_angle_, current_angle, angle_diff);


  // Parameters
  const double angular_speed_gain = 1.5;
  const double max_angular_speed = 0.5;
  const double max_forward_speed = 0.5;
  const double alignment_threshold = 0.1;

  // Align if needed
  if (std::abs(angle_diff) > alignment_threshold) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = std::clamp(angular_speed_gain * angle_diff, -max_angular_speed, max_angular_speed);
    RCLCPP_INFO(node_->get_logger(), "Rotating to align: angular.z=%.2f", cmd_vel.twist.angular.z);
    return cmd_vel;
  }


  // Drive forward
  double forward_speed = std::clamp(max_forward_speed * std::cos(angle_diff), 0.0, max_forward_speed);
  cmd_vel.twist.linear.x = forward_speed;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = std::clamp(angular_speed_gain * angle_diff, -max_angular_speed, max_angular_speed);

  RCLCPP_INFO(node_->get_logger(), "Driving forward: linear.x=%.2f, angular.z=%.2f",
              cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);

  return cmd_vel;
}

void AssertiveController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  RCLCPP_INFO(node_->get_logger(), "Speed limit set: %.2f (%s)",
              speed_limit, percentage ? "percent" : "absolute");
}

}  // namespace assertive_controller

PLUGINLIB_EXPORT_CLASS(assertive_controller::AssertiveController, nav2_core::Controller)
