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
#include "arpa_control/srv/plan_to_pose.hpp"
#include "arpa_control/srv/execute_plan.hpp"
#include "arpa_control/srv/stop_motion.hpp"
#include "arpa_control/srv/get_point_cloud.hpp"
#include "arpa_control/srv/get_pose_cost_matrix.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>


// lidar
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <mutex>
#include <thread>
#include <random>


class MotionControlNode : public rclcpp::Node
{
public:
  MotionControlNode(rclcpp::NodeOptions options);
  ~MotionControlNode();
  void init();
  void initMoveGroup();
  void dumpParams();
  std::shared_ptr<rclcpp::Node> getPlanSpinNode();
  std::shared_ptr<rclcpp::Node> getMoveSpinNode();


private:
  void planToPoseCallback(
      const std::shared_ptr<arpa_control::srv::PlanToPose::Request> request,
      std::shared_ptr<arpa_control::srv::PlanToPose::Response> response);

  void executePlanCallback(
    const std::shared_ptr<arpa_control::srv::ExecutePlan::Request> request,
    std::shared_ptr<arpa_control::srv::ExecutePlan::Response> response);

  void stopMotionCallback(
      const std::shared_ptr<arpa_control::srv::StopMotion::Request> request,
      std::shared_ptr<arpa_control::srv::StopMotion::Response> response);

  void getPoseCostMatrixCallback(
      const std::shared_ptr<arpa_control::srv::GetPoseCostMatrix::Request> request,
      std::shared_ptr<arpa_control::srv::GetPoseCostMatrix::Response> response);

  void updateDepthCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> m_move_group;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> m_planning_scene_monitor;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_scene_interface;
  rclcpp::Client<arpa_control::srv::GetPointCloud>::SharedPtr m_depth_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_depth_reset_client;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_update_depth_service;
  rclcpp::Service<arpa_control::srv::PlanToPose>::SharedPtr m_plan_to_pose_service;
  rclcpp::Service<arpa_control::srv::ExecutePlan>::SharedPtr m_execute_plan_service;
  rclcpp::Service<arpa_control::srv::StopMotion>::SharedPtr m_stop_motion_service;
  rclcpp::Service<arpa_control::srv::GetPoseCostMatrix>::SharedPtr m_get_pose_cost_matrix_service;

  moveit::planning_interface::MoveGroupInterface::Plan m_current_plan;
  geometry_msgs::msg::Pose m_current_target_pose;
  bool m_use_depth;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_transform_broadcaster;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  bool updateDepthMap(unsigned int timeout_ms = 2000);
  bool resetDepthMap(unsigned int timeout_ms = 2000);
  void initUpdateDepth();
  void checkRobotStateReady();
  double getConfigurationCost(
      const std::shared_ptr<moveit::core::RobotState>& current_state,
      const std::shared_ptr<moveit::core::RobotState>& target_state);
  std::vector<std::vector<double>> configureForPlanning(geometry_msgs::msg::Pose target_pose);
  /** Multi-objective IK seed selection (clearance, manipulability, joint distance, limit margin). */
  std::vector<std::vector<double>> configureForPlanningSpecial(geometry_msgs::msg::Pose target_pose);
  double getMinClearance(const std::shared_ptr<moveit::core::RobotState>& state);
  double getManipulability(const std::shared_ptr<moveit::core::RobotState>& state);
  double getJointLimitMargin(const std::shared_ptr<moveit::core::RobotState>& state);
  double getWeightedJointDistance(
      const std::shared_ptr<moveit::core::RobotState>& current_state,
      const std::shared_ptr<moveit::core::RobotState>& target_state);
  /** Returns 1.0 when actuator is at table centre (midpoint of range), 0.0 at limits. Used to prefer planning through middle of table. */
  double getActuatorCentreBonus(const std::shared_ptr<moveit::core::RobotState>& state);
  geometry_msgs::msg::PoseStamped poseToPlanningFrame(const geometry_msgs::msg::PoseStamped& pose_stamped);
  double computePairwiseCost(
      const geometry_msgs::msg::PoseStamped& src_pose,
      const geometry_msgs::msg::PoseStamped& tgt_pose,
      const moveit::core::JointModelGroup* jmg,
      const std::string& ee_link);
  void updateGoalMarker(const std::shared_ptr<moveit::core::RobotState>& state);
  /** Set path constraints to a flat cuboid corridor: long axis along segment, wide in XY (sides), flatter in Z. */
  void setCorridorPathConstraints(
      const Eigen::Vector3d& start_pos,
      const Eigen::Vector3d& end_pos,
      double padding,
      double cross_section_side,
      double cross_section_z,
      const geometry_msgs::msg::Quaternion& desired_orientation,
      const std::string& planning_frame,
      const std::string& ee_link);
  /** Publish a corridor box marker to RViz (rectangular corridor). */
  void publishCorridorMarker(
      const std::string& frame_id,
      const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation,
      double dim_x, double dim_y, double dim_z,
      bool is_flat_corridor);
  /** Remove the corridor marker from RViz (call before showing a new corridor or when planning ends). */
  void clearCorridorMarker();
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr m_goal_state_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_corridor_marker_pub;
  rclcpp::CallbackGroup::SharedPtr m_depth_client_group;
  float m_arm_padding;
  std::map<std::string, double> m_arm_padding_map;
  std::vector<std::string> m_arm_padding_links;
  rclcpp::TimerBase::SharedPtr m_init_timer;
  bool m_robot_state_ready = false;

  // Store the goal joint values from best_state for comparison after execution
  std::vector<double> m_goal_joint_values;

  // Joint weights for cost metrics: [linear actuator, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3].
  // Small actuator weight (0.2) biases IK seed selection toward seeds where the gantry stays near
  // its current position — corridor planning succeeds more reliably when gantry travel is minimised.
  const std::vector<double> m_joint_weights = {0.2, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // Dedicated node and thread for MoveGroupInterface
  rclcpp::Node::SharedPtr m_move_group_node;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr m_move_group_executor;
  std::thread m_move_group_thread;

  std::mt19937 m_rng;
  std::uniform_real_distribution<double> m_arm_noise_dist;

  /** When true, use multi-objective (MOGA-style) IK seed selection in planToPoseCallback. */
  bool m_special_logic;
};

#endif // __MOTION_CONTROL_NODE__