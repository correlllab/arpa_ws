#ifndef __MOTION_CONTROL_NODE__ 
#define __MOTION_CONTROL_NODE__ 

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "ur_manipulation/srv/plan_to_pose.hpp"
#include "ur_manipulation/srv/plan_to_joint.hpp"
#include "ur_manipulation/srv/execute_plan.hpp"
#include "ur_manipulation/srv/stop_motion.hpp"
#include "ur_manipulation/srv/get_point_cloud.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>


// lidar
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <mutex>


class MotionControlNode : public rclcpp::Node
{
public:
  MotionControlNode(rclcpp::NodeOptions options);
  void init();
  void initMoveGroup();
  std::shared_ptr<rclcpp::Node> getPlanSpinNode();
  std::shared_ptr<rclcpp::Node> getMoveSpinNode();


private:
  void planToPoseCallback(
      const std::shared_ptr<ur_manipulation::srv::PlanToPose::Request> request,
      std::shared_ptr<ur_manipulation::srv::PlanToPose::Response> response);

  void planToJointCallback(
      const std::shared_ptr<ur_manipulation::srv::PlanToJoint::Request> request,
      std::shared_ptr<ur_manipulation::srv::PlanToJoint::Response> response);

  void executePlanCallback(
    const std::shared_ptr<ur_manipulation::srv::ExecutePlan::Request> request,
    std::shared_ptr<ur_manipulation::srv::ExecutePlan::Response> response);

  void stopMotionCallback(
      const std::shared_ptr<ur_manipulation::srv::StopMotion::Request> request,
      std::shared_ptr<ur_manipulation::srv::StopMotion::Response> response);

  void updateDepthCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> 
  m_planning_scene_monitor;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_scene_interface;
  rclcpp::Client<ur_manipulation::srv::GetPointCloud>::SharedPtr m_depth_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_depth_reset_client;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_update_depth_service;
  rclcpp::Service<ur_manipulation::srv::PlanToPose>::SharedPtr m_plan_to_pose_service;
  rclcpp::Service<ur_manipulation::srv::PlanToJoint>::SharedPtr m_plan_to_joint_service;
  rclcpp::Service<ur_manipulation::srv::ExecutePlan>::SharedPtr m_execute_plan_service;
  rclcpp::Service<ur_manipulation::srv::StopMotion>::SharedPtr m_stop_motion_service;

  moveit::planning_interface::MoveGroupInterface::Plan m_current_plan;
  bool m_use_depth;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_transform_broadcaster;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  bool updateDepthMap(unsigned int timeout_ms = 2000);
  bool resetDepthMap(unsigned int timeout_ms = 2000);
  void initUpdateDepth();
  void checkRobotStateReady();
  rclcpp::CallbackGroup::SharedPtr m_depth_client_group;
  float m_arm_padding;
  std::map<std::string, double> m_arm_padding_map;
  std::vector<std::string> m_arm_padding_links;
  rclcpp::TimerBase::SharedPtr m_init_timer;
  bool m_robot_state_ready = false;

  // Helper function for relative motion planning - simplifies moving end-effector by delta
  geometry_msgs::msg::Pose planRelativeMotion(
      double dx = 0.0, double dy = 0.0, double dz = 0.0,
      double droll = 0.0, double dpitch = 0.0, double dyaw = 0.0);
};

#endif // __MOTION_CONTROL_NODE__