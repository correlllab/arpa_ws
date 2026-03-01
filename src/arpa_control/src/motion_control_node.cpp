#include "arpa_control/motion_control_node.hpp"
#include <rclcpp/exceptions.hpp>
#include <chrono>
#include <future>
#include <cmath>
#include <thread>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <limits>
#include <algorithm>
#include <numeric>
#include <random>
#include <moveit/robot_state/conversions.h>

MotionControlNode::MotionControlNode(rclcpp::NodeOptions options)
    : Node("motion_control_node", options)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Constructor Init");
  m_rng = std::mt19937{std::random_device{}()};
  m_arm_noise_dist = std::uniform_real_distribution<double>{-0.2, 0.2};
  m_plan_to_pose_service = this->create_service<arpa_control::srv::PlanToPose>(
      "plan_to_pose",
      std::bind(&MotionControlNode::planToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_execute_plan_service = this->create_service<arpa_control::srv::ExecutePlan>(
      "execute_plan",
      std::bind(&MotionControlNode::executePlanCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_stop_motion_service = this->create_service<arpa_control::srv::StopMotion>(
      "stop_motion",
      std::bind(&MotionControlNode::stopMotionCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_get_pose_cost_matrix_service = this->create_service<arpa_control::srv::GetPoseCostMatrix>(
      "get_pose_cost_matrix",
      std::bind(&MotionControlNode::getPoseCostMatrixCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_update_depth_service = this->create_service<std_srvs::srv::Trigger>(
      "update_depth",
      std::bind(&MotionControlNode::updateDepthCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_depth_reset_client = this->create_client<std_srvs::srv::Trigger>("arm_pointcloud");

  m_depth_client_group =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  m_depth_client = this->create_client<arpa_control::srv::GetPointCloud>(
    "/pointcloud_accumulator/get_arm_pointcloud",
    rmw_qos_profile_services_default,
      m_depth_client_group);
  
  // Static TF Broacaster
  m_static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  m_goal_state_pub = this->create_publisher<moveit_msgs::msg::DisplayRobotState>(
      "/goal_robot_state",
      rclcpp::QoS(1));
  m_corridor_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/planning_corridor_marker",
      rclcpp::QoS(1));

  m_use_depth = false;
  try {
    m_special_logic = this->get_parameter("use_special_logic").as_bool();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
    this->declare_parameter("use_special_logic", true);
    m_special_logic = this->get_parameter("use_special_logic").as_bool();
  }
  RCLCPP_INFO(get_logger(), "use_special_logic (m_special_logic): %s", m_special_logic ? "true" : "false");
  //TODO verify octomap resolution is being used
  this->declare_parameter("octomap_resolution", 0.03);
  this->declare_parameter("arm_padding", 0.015);
  m_arm_padding = this->get_parameter("arm_padding").as_double();
  m_arm_padding_links = {"forearm_link", "shoulder_link", "upper_arm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "tool_holder_link", "runner_link", "tool_head_link"};
  for(auto link : m_arm_padding_links) {
    m_arm_padding_map[link] = m_arm_padding;
  }

  // IK seed solver tuning (more/diverse seeds often yield plans that pass post-processing)
  try {
    this->declare_parameter("ik_seed_timeout", 0.2);  // seconds per setFromIK attempt (increase if IK often fails)
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("ik_seed_actuator_offset_step", 0.1);  // actuator offset step (m), e.g. 0.05 for finer
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("ik_seed_actuator_offset_max", 1.0);  // max actuator offset (m) from current
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("ik_seed_perturbation_attempts", 1);   // extra IK tries per offset with perturbed arm seed (0=off)
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("ik_seed_min_clearance", 0.60);  // min distance wrist–actuator (m); lower allows more solutions
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("ik_seed_table_centre_bonus", 0.25);  // cost reduction / score bonus when actuator near table centre (linear actuator + two-link arm planning)
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("corridor_cross_section_z", 0.10);  // flat corridor height (Z half-extent); side extent from corridor_cross_section
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }
  try {
    this->declare_parameter("corridor_z_floor_tolerance", 0.05);  // EE may go at most this far below the lower endpoint Z (m); negative disables
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) { /* from launch */ }

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  // Create dedicated node for MoveGroupInterface
  m_move_group_node = rclcpp::Node::make_shared("move_group_interface_node", options);

  // Create executor and add the dedicated node
  m_move_group_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  m_move_group_executor->add_node(m_move_group_node);

  // Start spinning the dedicated node in a separate thread
  m_move_group_thread = std::thread([this]() {
    m_move_group_executor->spin();
  });

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Constructor Initialized");
}

MotionControlNode::~MotionControlNode()
{
  // Shutdown the executor and join the thread
  m_move_group_executor->cancel();
  if (m_move_group_thread.joinable()) {
    m_move_group_thread.join();
  }
}

void MotionControlNode::init()
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Init()");
  m_planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  // Use the dedicated node for MoveGroupInterface (has its own spinning thread)
  m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    m_move_group_node, "ur16e_on_gantry");
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Init() END");
}

void MotionControlNode::initMoveGroup()
{
  /*
  https://docs.ros.org/en/lunar/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html
  */
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() START");

  m_move_group->startStateMonitor(2.5);
  m_move_group->setPlanningPipelineId("move_group");

  m_move_group->setPlannerId("RRTConnectkConfigDefault");
  // m_move_group->setPlannerId("RRTstarkConfigDefault");
  // RViz defaults: 5s, 10 attempts, 0.1 vel scaling, 0.1 accel scaling.
  // Give the corridor-constrained RRT more time to find a path, especially for
  // corner edges and middle (long segments, tight turns).
  m_move_group->setPlanningTime(28.0);
  m_move_group->setNumPlanningAttempts(28);
  m_move_group->setMaxVelocityScalingFactor(0.1);
  m_move_group->setMaxAccelerationScalingFactor(0.1);
  m_move_group->setGoalPositionTolerance(0.001);  // 1mm tolerance
  m_move_group->setGoalOrientationTolerance(0.001);  // ~0.057 degrees
  m_move_group->setGoalJointTolerance(0.001);  // 0.001 rad (~0.057 degrees) per joint

  m_move_group->allowReplanning(true);
  m_move_group->setReplanAttempts(5);
  m_move_group->setReplanDelay(0.15);  // seconds between replans

  // Allow sensor updates during planning
  // m_move_group->allowLooking(true);

  // // Orientation constraint: keep tool pointing down during motion
  // // Axis-aligned: RPY (180°, 0°, 90°) - tool pointing down (-Z), Y-axis forward
  // moveit_msgs::msg::Constraints path_constraints;
  // moveit_msgs::msg::OrientationConstraint ocm;
  // ocm.link_name = m_move_group->getEndEffectorLink();
  // ocm.header.frame_id = "floor_link";
  // // Quaternion (xyzw): [0.7071068, 0.7071068, 0, 0]
  // ocm.orientation.x = 0.7071068;
  // ocm.orientation.y = 0.7071068;
  // ocm.orientation.z = 0.0;
  // ocm.orientation.w = 0.0;
  // ocm.absolute_x_axis_tolerance = 3.14/2;  // radians of allowed deviation (~29°)
  // ocm.absolute_y_axis_tolerance = 3.14/2;
  // ocm.absolute_z_axis_tolerance = 2*3.14; // free rotation around Z (tool axis)
  // ocm.weight = 1.0;
  // path_constraints.orientation_constraints.push_back(ocm);
  // m_move_group->setPathConstraints(path_constraints);

  // Other useful settings (commented out for reference)
  // m_move_group->setGoalJointTolerance(0.01);           // joint-space tolerance (radians)
  // m_move_group->setGoalTolerance(0.01);                // sets position, orientation, AND joint tolerances
  // m_move_group->setWorkspace(-1.0, -2.0, 0.5, 3.0, 2.0, 2.0);  // bounding box for end-effector
  // m_move_group->setPoseReferenceFrame("world");   // frame for pose targets
  // m_move_group->setEndEffectorLink("tool0");           // which link to plan for
  // m_move_group->setSupportSurfaceName("table");        // for pick/place operations
  // m_move_group->clearPathConstraints();                // remove constraints

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() END");
  dumpParams();
}

void MotionControlNode::dumpParams()
{
  RCLCPP_INFO(get_logger(), "\n\n========== MoveGroupInterface Parameters ==========");
  RCLCPP_INFO(get_logger(), "  Group name:                %s", m_move_group->getName().c_str());
  RCLCPP_INFO(get_logger(), "  Planning frame:            %s", m_move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(get_logger(), "  Pose reference frame:      %s", m_move_group->getPoseReferenceFrame().c_str());
  RCLCPP_INFO(get_logger(), "  End effector link:         %s", m_move_group->getEndEffectorLink().c_str());
  RCLCPP_INFO(get_logger(), "  End effector:              %s", m_move_group->getEndEffector().c_str());
  RCLCPP_INFO(get_logger(), "  Planner ID:                %s", m_move_group->getPlannerId().c_str());
  RCLCPP_INFO(get_logger(), "  Planning time:             %.2f s", m_move_group->getPlanningTime());
  RCLCPP_INFO(get_logger(), "  Goal position tolerance:   %.4f", m_move_group->getGoalPositionTolerance());
  RCLCPP_INFO(get_logger(), "  Goal orientation tolerance: %.4f", m_move_group->getGoalOrientationTolerance());
  RCLCPP_INFO(get_logger(), "  Goal joint tolerance:      %.4f", m_move_group->getGoalJointTolerance());
  RCLCPP_INFO(get_logger(), "  Variable count:            %u", m_move_group->getVariableCount());

  auto joints = m_move_group->getJoints();
  std::string joints_str;
  for (const auto& j : joints) {
    joints_str += j + ", ";
  }
  RCLCPP_INFO(get_logger(), "  Joints:                    %s", joints_str.c_str());

  auto active_joints = m_move_group->getActiveJoints();
  std::string active_str;
  for (const auto& j : active_joints) {
    active_str += j + ", ";
  }
  RCLCPP_INFO(get_logger(), "  Active joints:             %s", active_str.c_str());
  RCLCPP_INFO(get_logger(), "\n====================================================\n\n");
}

double MotionControlNode::getConfigurationCost(
    const std::shared_ptr<moveit::core::RobotState>& current_state,
    const std::shared_ptr<moveit::core::RobotState>& target_state)
{
  // Check if joints are within valid limits for both states
  const auto* jmg = target_state->getJointModelGroup(m_move_group->getName());

  if (!target_state->satisfiesBounds(jmg)) {
    RCLCPP_WARN(get_logger(), "Target state has joints out of valid range");
    return std::numeric_limits<double>::infinity();
  }

  // Check collision: Euclidean distance between linear actuator plate and elbow
  const Eigen::Isometry3d& actuator_tf =
      target_state->getGlobalLinkTransform("linear_actuator_plate_link");
  const Eigen::Isometry3d& elbow_tf =
      target_state->getGlobalLinkTransform("wrist_3_link");

  double ee_distance = (actuator_tf.translation() - elbow_tf.translation()).norm();
  const double min_clearance = this->get_parameter("ik_seed_min_clearance").as_double();
  if (ee_distance < min_clearance) {
    RCLCPP_WARN(get_logger(),
        "EE too close to linear actuator plate: %.3f m (min %.2f m)", ee_distance, min_clearance);
    return std::numeric_limits<double>::infinity();
  }

  // Weighted joint distance from current to target
  std::vector<double> current_values, target_values;
  current_state->copyJointGroupPositions(jmg, current_values);
  target_state->copyJointGroupPositions(jmg, target_values);

  // Weights per joint (linear actuator, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
  double joint_cost = 0.0;
  for (size_t i = 0; i < current_values.size(); ++i) {
    double diff = target_values[i] - current_values[i];
    joint_cost += m_joint_weights[i] * diff * diff;
  }
  joint_cost = std::sqrt(joint_cost);

  // Add proximity penalty: penalize configurations where wrist is close to actuator
  double actuator_wrist_distance = ee_distance;  // Using wrist_3_link distance (same as elbow check)
  double proximity_penalty = 0.1 / actuator_wrist_distance;

  double total_cost = joint_cost + proximity_penalty;

  // Prefer planning through middle of table: reduce cost when linear actuator is near centre (and arm two-link posture is used)
  double centre_bonus = getActuatorCentreBonus(target_state);
  const double table_centre_weight = this->get_parameter("ik_seed_table_centre_bonus").as_double();
  total_cost -= table_centre_weight * centre_bonus;

  if (total_cost < 0.0) total_cost = 0.0;

  return total_cost;
}

void MotionControlNode::updateGoalMarker(const std::shared_ptr<moveit::core::RobotState>& state)
{
  // Publish DisplayRobotState with joint values
  moveit_msgs::msg::DisplayRobotState display_state;
  display_state.state.is_diff = false;

  // Get joint names and positions from the robot state
  const auto* jmg = state->getJointModelGroup(m_move_group->getName());
  std::vector<double> joint_positions;
  state->copyJointGroupPositions(jmg, joint_positions);
  const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();

  // Populate the joint state
  display_state.state.joint_state.header.stamp = now();
  display_state.state.joint_state.header.frame_id = m_move_group->getPlanningFrame();
  display_state.state.joint_state.name = joint_names;
  display_state.state.joint_state.position = joint_positions;

  // Get all link names from the robot model and color them green
  const std::vector<std::string>& link_names = state->getRobotModel()->getLinkModelNames();
  for (const auto& link_name : link_names) {
    moveit_msgs::msg::ObjectColor obj_color;
    obj_color.id = link_name;
    obj_color.color.r = 0.0;
    obj_color.color.g = 1.0;
    obj_color.color.b = 0.0;
    obj_color.color.a = 0.8;
    display_state.highlight_links.push_back(obj_color);
  }

  m_goal_state_pub->publish(display_state);
}

void MotionControlNode::setCorridorPathConstraints(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    double padding,
    double cross_section_side,
    double cross_section_z,
    const geometry_msgs::msg::Quaternion& desired_orientation,
    const std::string& planning_frame,
    const std::string& ee_link)
{
  Eigen::Vector3d diff = end_pos - start_pos;
  double seg_len = diff.norm();
  if (seg_len < 1e-6) return;

  // Orient corridor: long axis along segment (horizontal), wide along sides (XY panning), flat in Z.
  // Box frame: X = along segment, Y = sideways in XY (perp_xy), Z = world up.
  Eigen::Vector3d dir_xy(diff.x(), diff.y(), 0.0);
  double len_xy = dir_xy.norm();
  if (len_xy < 1e-6) {
    dir_xy = Eigen::Vector3d::UnitX();
  } else {
    dir_xy /= len_xy;
  }
  Eigen::Vector3d perp_xy(-dir_xy.y(), dir_xy.x(), 0.0);
  Eigen::Vector3d z_axis(0.0, 0.0, 1.0);

  const double validation_margin = 1.35;
  double length_along = (seg_len + 2.0 * padding) * validation_margin;
  const double width_side = std::max(2.0 * cross_section_side * validation_margin, 0.05);
  const double width_z = std::max(2.0 * cross_section_z * validation_margin, 0.05);
  length_along = std::max(length_along, std::max(width_side, width_z));

  Eigen::Vector3d mid = start_pos + 0.5 * diff;
  // Rotation from box frame to world: box X -> dir_xy, box Y -> perp_xy, box Z -> z_axis
  Eigen::Matrix3d R;
  R.col(0) = dir_xy;
  R.col(1) = perp_xy;
  R.col(2) = z_axis;
  Eigen::Quaterniond quat(R);

  moveit_msgs::msg::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = planning_frame;
  pos_constraint.link_name = ee_link;
  pos_constraint.target_point_offset.x = 0.0;
  pos_constraint.target_point_offset.y = 0.0;
  pos_constraint.target_point_offset.z = 0.0;
  pos_constraint.weight = 1.0;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = length_along;
  box.dimensions[1] = width_side;
  box.dimensions[2] = width_z;

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = mid.x();
  box_pose.position.y = mid.y();
  box_pose.position.z = mid.z();
  box_pose.orientation.x = quat.x();
  box_pose.orientation.y = quat.y();
  box_pose.orientation.z = quat.z();
  box_pose.orientation.w = quat.w();

  pos_constraint.constraint_region.primitives.push_back(box);
  pos_constraint.constraint_region.primitive_poses.push_back(box_pose);

  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = ee_link;
  ocm.header.frame_id = planning_frame;
  ocm.orientation = desired_orientation;
  ocm.absolute_x_axis_tolerance = 0.5;
  ocm.absolute_y_axis_tolerance = 0.5;
  ocm.absolute_z_axis_tolerance = 2.0 * M_PI;
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.position_constraints.push_back(pos_constraint);
  path_constraints.orientation_constraints.push_back(ocm);

  // Z-floor constraint: prevent EE from dipping below the scan plane.
  // Modelled as a very large world-aligned box whose bottom is at z_floor.
  // Stops RRT spending budget on paths that route beneath the gantry structure.
  const double z_floor_tol = this->get_parameter("corridor_z_floor_tolerance").as_double();
  if (z_floor_tol >= 0.0) {
    const double z_floor = std::min(start_pos.z(), end_pos.z()) - z_floor_tol;
    const double box_half_horiz = 50.0;  // effectively unconstrained in X/Y
    const double box_half_vert  = 50.0;  // large upward extent
    moveit_msgs::msg::PositionConstraint z_floor_con;
    z_floor_con.header.frame_id = planning_frame;
    z_floor_con.link_name = ee_link;
    z_floor_con.weight = 1.0;
    shape_msgs::msg::SolidPrimitive z_box;
    z_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    z_box.dimensions = {2.0 * box_half_horiz, 2.0 * box_half_horiz, 2.0 * box_half_vert};
    geometry_msgs::msg::Pose z_pose;
    z_pose.position.x = 0.0;
    z_pose.position.y = 0.0;
    z_pose.position.z = z_floor + box_half_vert;  // centre is 50 m above floor
    z_pose.orientation.w = 1.0;                   // identity — world-aligned
    z_floor_con.constraint_region.primitives.push_back(z_box);
    z_floor_con.constraint_region.primitive_poses.push_back(z_pose);
    path_constraints.position_constraints.push_back(z_floor_con);
  }

  m_move_group->setPathConstraints(path_constraints);

  publishCorridorMarker(planning_frame, mid, quat, length_along, width_side, width_z, true);
}

void MotionControlNode::publishCorridorMarker(
    const std::string& frame_id,
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    double dim_x, double dim_y, double dim_z,
    bool is_flat_corridor)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = now();
  m.ns = "planning_corridor";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = position.x();
  m.pose.position.y = position.y();
  m.pose.position.z = position.z();
  m.pose.orientation.x = orientation.x();
  m.pose.orientation.y = orientation.y();
  m.pose.orientation.z = orientation.z();
  m.pose.orientation.w = orientation.w();
  m.scale.x = dim_x;
  m.scale.y = dim_y;
  m.scale.z = dim_z;
  if (is_flat_corridor) {
    m.color.r = 0.2f;
    m.color.g = 0.4f;
    m.color.b = 1.0f;
  } else {
    m.color.r = 0.2f;
    m.color.g = 1.0f;
    m.color.b = 0.2f;
  }
  m.color.a = 0.35f;
  m_corridor_marker_pub->publish(m);
}

void MotionControlNode::clearCorridorMarker()
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = now();
  m.ns = "planning_corridor";
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::DELETE;
  m_corridor_marker_pub->publish(m);
}

std::vector<std::vector<double>> MotionControlNode::configureForPlanning(geometry_msgs::msg::Pose target_pose)
{
  auto current_state = m_move_group->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state");
    return {};
  }

  const auto* jmg = current_state->getJointModelGroup(m_move_group->getName());
  const std::string& ee_link = m_move_group->getEndEffectorLink();
  const std::string actuator_joint = "linear_actuator_to_linear_actuator_plate_joint";

  double original_actuator_pos = *current_state->getJointPositions(actuator_joint);
  RCLCPP_INFO(get_logger(), "Current linear actuator position: %.3f m", original_actuator_pos);

  const double ik_timeout = this->get_parameter("ik_seed_timeout").as_double();
  const double offset_step = this->get_parameter("ik_seed_actuator_offset_step").as_double();
  const double offset_max = this->get_parameter("ik_seed_actuator_offset_max").as_double();
  const int perturbation_attempts = std::max(0, static_cast<int>(this->get_parameter("ik_seed_perturbation_attempts").as_int()));

  // Clamp actuator seed to joint limits so we never pass an invalid seed (important for centre poses)
  double actuator_min = -std::numeric_limits<double>::infinity();
  double actuator_max = std::numeric_limits<double>::infinity();
  const moveit::core::RobotModel& model = *current_state->getRobotModel();
  const moveit::core::JointModel* jm = model.getJointModel(actuator_joint);
  if (jm && jm->getVariableCount() > 0) {
    const std::string& var_name = jm->getVariableNames()[0];
    const moveit::core::VariableBounds& b = model.getVariableBounds(var_name);
    if (b.position_bounded_) {
      actuator_min = b.min_position_;
      actuator_max = b.max_position_;
    }
  }

  std::vector<std::vector<double>> all_solutions;
  std::vector<double> all_costs;

  // Build actuator seeds: target-informed first, then midpoint, current, ±offsets, hard limits.
  std::vector<double> actuator_seeds;
  const bool has_limits = std::isfinite(actuator_min) && std::isfinite(actuator_max);

  // Target-informed seeds: estimate required gantry position from target x-coordinate.
  // Actuator axis is (-1,0,0), origin at x≈1.0244, so actuator_pos ≈ 1.0244 - target.x.
  // Seeds near this estimate give the IK solver the best starting region; the corridor
  // re-ranking in planToPoseCallback then promotes the closest-to-estimate solutions first.
  if (has_limits) {
    const double target_actuator_est = std::clamp(
        1.0244 - target_pose.position.x, actuator_min, actuator_max);
    for (double off : {0.0, -0.15, 0.15, -0.30, 0.30}) {
      double p = std::clamp(target_actuator_est + off, actuator_min, actuator_max);
      if (std::find_if(actuator_seeds.begin(), actuator_seeds.end(),
          [p](double v) { return std::abs(v - p) < 1e-6; }) == actuator_seeds.end())
        actuator_seeds.push_back(p);
    }
  }

  if (has_limits) {
    double actuator_mid = 0.5 * (actuator_min + actuator_max);
    if (std::find_if(actuator_seeds.begin(), actuator_seeds.end(),
        [actuator_mid](double v) { return std::abs(v - actuator_mid) < 1e-6; }) == actuator_seeds.end())
      actuator_seeds.push_back(actuator_mid);
  }
  if (std::find_if(actuator_seeds.begin(), actuator_seeds.end(),
      [original_actuator_pos](double v) { return std::abs(v - original_actuator_pos) < 1e-6; }) == actuator_seeds.end())
    actuator_seeds.push_back(original_actuator_pos);
  for (double o = offset_step; o <= offset_max + 1e-9; o += offset_step) {
    double p_plus = std::clamp(original_actuator_pos + o, actuator_min, actuator_max);
    double p_minus = std::clamp(original_actuator_pos - o, actuator_min, actuator_max);
    if (std::find_if(actuator_seeds.begin(), actuator_seeds.end(), [p_plus](double v) { return std::abs(v - p_plus) < 1e-6; }) == actuator_seeds.end())
      actuator_seeds.push_back(p_plus);
    if (std::find_if(actuator_seeds.begin(), actuator_seeds.end(), [p_minus](double v) { return std::abs(v - p_minus) < 1e-6; }) == actuator_seeds.end())
      actuator_seeds.push_back(p_minus);
  }
  if (std::isfinite(actuator_min) && std::find_if(actuator_seeds.begin(), actuator_seeds.end(), [actuator_min](double v) { return std::abs(v - actuator_min) < 1e-6; }) == actuator_seeds.end())
    actuator_seeds.push_back(actuator_min);
  if (std::isfinite(actuator_max) && std::find_if(actuator_seeds.begin(), actuator_seeds.end(), [actuator_max](double v) { return std::abs(v - actuator_max) < 1e-6; }) == actuator_seeds.end())
    actuator_seeds.push_back(actuator_max);

  const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
  const size_t num_joints = joint_names.size();
  size_t idx_pan = num_joints, idx_lift = num_joints;
  for (size_t i = 0; i < num_joints; ++i) {
    if (joint_names[i] == "shoulder_pan_joint") idx_pan = i;
    if (joint_names[i] == "shoulder_lift_joint") idx_lift = i;
  }
  const bool has_two_link_seeds = (idx_pan < num_joints && idx_lift < num_joints);
  // Offsets (rad) for shoulder_pan, shoulder_lift to bias table-centre reaches (linear actuator + two links)
  const std::vector<std::pair<double, double>> arm_centre_offsets = {{0.0, 0.0}, {0.0, -0.4}, {0.0, -0.8}};

  for (double seed_pos : actuator_seeds) {
    bool is_midpoint_seed = has_limits && (std::abs(seed_pos - 0.5 * (actuator_min + actuator_max)) < 1e-6);
    const size_t arm_iters = (is_midpoint_seed && has_two_link_seeds) ? arm_centre_offsets.size() : 1u;

    for (size_t arm_idx = 0; arm_idx < arm_iters; ++arm_idx) {
      for (int perturb = 0; perturb <= perturbation_attempts; ++perturb) {
        auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
        seed_state->setJointPositions(actuator_joint, &seed_pos);
        if (arm_iters > 1 && arm_idx < arm_centre_offsets.size()) {
          double pan = current_state->getVariablePosition(joint_names[idx_pan]) + arm_centre_offsets[arm_idx].first;
          double lift = current_state->getVariablePosition(joint_names[idx_lift]) + arm_centre_offsets[arm_idx].second;
          seed_state->setVariablePosition(joint_names[idx_pan], pan);
          seed_state->setVariablePosition(joint_names[idx_lift], lift);
        }
        if (perturb > 0 && num_joints > 1) {
          for (size_t i = 1; i < num_joints; ++i) {
            double v = seed_state->getVariablePosition(joint_names[i]);
            v += m_arm_noise_dist(m_rng) * 0.15;  // small perturbation to get different IK branch
            seed_state->setVariablePosition(joint_names[i], v);
          }
          seed_state->update();
        } else {
          seed_state->update();
        }

        if (!seed_state->setFromIK(jmg, target_pose, ee_link, ik_timeout)) {
          RCLCPP_DEBUG(get_logger(), "IK failed for actuator seed %.2f%s", seed_pos, perturb ? " (perturbed)" : "");
          continue;
        }
        seed_state->update();

        double cost = getConfigurationCost(current_state, seed_state);
        if (std::isinf(cost)) continue;

        std::vector<double> joint_positions;
        seed_state->copyJointGroupPositions(jmg, joint_positions);
        all_solutions.push_back(joint_positions);
        all_costs.push_back(cost);
      }
    }
  }

  if (all_solutions.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid IK solution found (%zu actuator seeds, perturbs=%d)",
        actuator_seeds.size(), perturbation_attempts);
    return {};
  }

  // Sort by cost ascending
  std::vector<size_t> indices(all_solutions.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(),
      [&](size_t a, size_t b) { return all_costs[a] < all_costs[b]; });

  std::vector<std::vector<double>> sorted_solutions;
  sorted_solutions.reserve(indices.size());
  for (size_t idx : indices) {
    sorted_solutions.push_back(all_solutions[idx]);
  }

  RCLCPP_INFO(get_logger(), "Found %zu IK solutions (best cost=%.4f, worst cost=%.4f)",
      sorted_solutions.size(), all_costs[indices.front()], all_costs[indices.back()]);

  return sorted_solutions;
}

double MotionControlNode::getMinClearance(const std::shared_ptr<moveit::core::RobotState>& state)
{
  const Eigen::Isometry3d& actuator_tf =
      state->getGlobalLinkTransform("linear_actuator_plate_link");
  const Eigen::Isometry3d& wrist_tf =
      state->getGlobalLinkTransform("wrist_3_link");
  return (actuator_tf.translation() - wrist_tf.translation()).norm();
}

double MotionControlNode::getManipulability(const std::shared_ptr<moveit::core::RobotState>& state)
{
  const auto* jmg = state->getJointModelGroup(m_move_group->getName());
  const std::string& ee_link_name = m_move_group->getEndEffectorLink();
  const moveit::core::LinkModel* ee_link = state->getLinkModel(ee_link_name);
  if (!jmg || !ee_link) return 0.0;

  Eigen::MatrixXd jacobian;
  state->getJacobian(jmg, ee_link, Eigen::Vector3d::Zero(), jacobian, false);
  if (jacobian.rows() == 0 || jacobian.cols() == 0) return 0.0;

  Eigen::MatrixXd jjt = jacobian * jacobian.transpose();
  double det = jjt.determinant();
  if (det <= 0.0) return 0.0;
  return std::sqrt(det);
}

double MotionControlNode::getJointLimitMargin(const std::shared_ptr<moveit::core::RobotState>& state)
{
  const auto* jmg = state->getJointModelGroup(m_move_group->getName());
  if (!jmg) return 0.0;

  const moveit::core::RobotModel& model = *state->getRobotModel();
  const std::vector<std::string>& var_names = jmg->getActiveJointModelNames();
  double min_margin = std::numeric_limits<double>::infinity();

  for (const std::string& name : var_names) {
    const moveit::core::VariableBounds& b = model.getVariableBounds(name);
    if (!b.position_bounded_) continue;
    double pos = state->getVariablePosition(name);
    double margin_lo = pos - b.min_position_;
    double margin_hi = b.max_position_ - pos;
    double margin = std::min(margin_lo, margin_hi);
    if (margin < min_margin) min_margin = margin;
  }
  return std::isinf(min_margin) ? 0.0 : min_margin;
}

double MotionControlNode::getWeightedJointDistance(
    const std::shared_ptr<moveit::core::RobotState>& current_state,
    const std::shared_ptr<moveit::core::RobotState>& target_state)
{
  const auto* jmg = target_state->getJointModelGroup(m_move_group->getName());
  std::vector<double> current_values, target_values;
  current_state->copyJointGroupPositions(jmg, current_values);
  target_state->copyJointGroupPositions(jmg, target_values);
  double sum = 0.0;
  for (size_t i = 0; i < current_values.size(); ++i) {
    double diff = target_values[i] - current_values[i];
    sum += m_joint_weights[i] * diff * diff;
  }
  return std::sqrt(sum);
}

double MotionControlNode::getActuatorCentreBonus(const std::shared_ptr<moveit::core::RobotState>& state)
{
  const std::string actuator_joint = "linear_actuator_to_linear_actuator_plate_joint";
  const moveit::core::RobotModel& model = *state->getRobotModel();
  const moveit::core::JointModel* jm = model.getJointModel(actuator_joint);
  if (!jm || jm->getVariableCount() == 0) return 0.0;
  const std::string& var_name = jm->getVariableNames()[0];
  const moveit::core::VariableBounds& b = model.getVariableBounds(var_name);
  if (!b.position_bounded_) return 0.0;
  double pos = state->getVariablePosition(var_name);
  double mid = 0.5 * (b.min_position_ + b.max_position_);
  double half_range = 0.5 * (b.max_position_ - b.min_position_);
  if (half_range < 1e-9) return 1.0;
  double dist_from_mid = std::abs(pos - mid);
  return 1.0 - std::min(1.0, dist_from_mid / half_range);
}

std::vector<std::vector<double>> MotionControlNode::configureForPlanningSpecial(
    geometry_msgs::msg::Pose target_pose)
{
  auto current_state = m_move_group->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state");
    return {};
  }

  const auto* jmg = current_state->getJointModelGroup(m_move_group->getName());
  const std::string& ee_link = m_move_group->getEndEffectorLink();
  const std::string actuator_joint = "linear_actuator_to_linear_actuator_plate_joint";
  double original_actuator_pos = *current_state->getJointPositions(actuator_joint);

  const double ik_timeout = this->get_parameter("ik_seed_timeout").as_double();
  const double offset_step = this->get_parameter("ik_seed_actuator_offset_step").as_double();
  const double offset_max = this->get_parameter("ik_seed_actuator_offset_max").as_double();
  const int perturbation_attempts = std::max(0, static_cast<int>(this->get_parameter("ik_seed_perturbation_attempts").as_int()));

  double actuator_min = -std::numeric_limits<double>::infinity();
  double actuator_max = std::numeric_limits<double>::infinity();
  const moveit::core::RobotModel& model_spec = *current_state->getRobotModel();
  const moveit::core::JointModel* jm_spec = model_spec.getJointModel(actuator_joint);
  if (jm_spec && jm_spec->getVariableCount() > 0) {
    const std::string& var_name_spec = jm_spec->getVariableNames()[0];
    const moveit::core::VariableBounds& b = model_spec.getVariableBounds(var_name_spec);
    if (b.position_bounded_) {
      actuator_min = b.min_position_;
      actuator_max = b.max_position_;
    }
  }

  std::vector<double> actuator_seeds_spec;
  const bool has_limits_spec = std::isfinite(actuator_min) && std::isfinite(actuator_max);
  if (has_limits_spec) {
    double actuator_mid_spec = 0.5 * (actuator_min + actuator_max);
    actuator_seeds_spec.push_back(actuator_mid_spec);
  }
  actuator_seeds_spec.push_back(original_actuator_pos);
  for (double o = offset_step; o <= offset_max + 1e-9; o += offset_step) {
    double p_plus = std::clamp(original_actuator_pos + o, actuator_min, actuator_max);
    double p_minus = std::clamp(original_actuator_pos - o, actuator_min, actuator_max);
    if (std::find_if(actuator_seeds_spec.begin(), actuator_seeds_spec.end(), [p_plus](double v) { return std::abs(v - p_plus) < 1e-6; }) == actuator_seeds_spec.end())
      actuator_seeds_spec.push_back(p_plus);
    if (std::find_if(actuator_seeds_spec.begin(), actuator_seeds_spec.end(), [p_minus](double v) { return std::abs(v - p_minus) < 1e-6; }) == actuator_seeds_spec.end())
      actuator_seeds_spec.push_back(p_minus);
  }
  if (std::isfinite(actuator_min) && std::find_if(actuator_seeds_spec.begin(), actuator_seeds_spec.end(), [actuator_min](double v) { return std::abs(v - actuator_min) < 1e-6; }) == actuator_seeds_spec.end())
    actuator_seeds_spec.push_back(actuator_min);
  if (std::isfinite(actuator_max) && std::find_if(actuator_seeds_spec.begin(), actuator_seeds_spec.end(), [actuator_max](double v) { return std::abs(v - actuator_max) < 1e-6; }) == actuator_seeds_spec.end())
    actuator_seeds_spec.push_back(actuator_max);

  const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
  const size_t num_joints = joint_names.size();
  size_t idx_pan_spec = num_joints, idx_lift_spec = num_joints;
  for (size_t i = 0; i < num_joints; ++i) {
    if (joint_names[i] == "shoulder_pan_joint") idx_pan_spec = i;
    if (joint_names[i] == "shoulder_lift_joint") idx_lift_spec = i;
  }
  const bool has_two_link_spec = (idx_pan_spec < num_joints && idx_lift_spec < num_joints);
  const std::vector<std::pair<double, double>> arm_centre_offsets_spec = {{0.0, 0.0}, {0.0, -0.4}, {0.0, -0.8}};

  struct ScoredSolution { std::vector<double> joints; double score; };
  std::vector<ScoredSolution> scored;

  for (double seed_pos : actuator_seeds_spec) {
    bool is_midpoint_spec = has_limits_spec && (std::abs(seed_pos - 0.5 * (actuator_min + actuator_max)) < 1e-6);
    const size_t arm_iters_spec = (is_midpoint_spec && has_two_link_spec) ? arm_centre_offsets_spec.size() : 1u;

    for (size_t arm_idx = 0; arm_idx < arm_iters_spec; ++arm_idx) {
      for (int perturb = 0; perturb <= perturbation_attempts; ++perturb) {
        auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
        seed_state->setJointPositions(actuator_joint, &seed_pos);
        if (arm_iters_spec > 1 && arm_idx < arm_centre_offsets_spec.size()) {
          double pan = current_state->getVariablePosition(joint_names[idx_pan_spec]) + arm_centre_offsets_spec[arm_idx].first;
          double lift = current_state->getVariablePosition(joint_names[idx_lift_spec]) + arm_centre_offsets_spec[arm_idx].second;
          seed_state->setVariablePosition(joint_names[idx_pan_spec], pan);
          seed_state->setVariablePosition(joint_names[idx_lift_spec], lift);
        }
        if (perturb > 0 && num_joints > 1) {
          for (size_t i = 1; i < num_joints; ++i) {
            double v = seed_state->getVariablePosition(joint_names[i]);
            v += m_arm_noise_dist(m_rng) * 0.15;
            seed_state->setVariablePosition(joint_names[i], v);
          }
          seed_state->update();
        } else {
          seed_state->update();
        }
        if (!seed_state->setFromIK(jmg, target_pose, ee_link, ik_timeout)) continue;
        seed_state->update();

        if (getConfigurationCost(current_state, seed_state) == std::numeric_limits<double>::infinity())
          continue;

      double clearance = getMinClearance(seed_state);
      double manipulability = getManipulability(seed_state);
      double joint_dist = getWeightedJointDistance(current_state, seed_state);
      double limit_margin = getJointLimitMargin(seed_state);
      double centre_bonus = getActuatorCentreBonus(seed_state);
      const double w_centre = this->get_parameter("ik_seed_table_centre_bonus").as_double();
      double actuator_moved = std::abs(seed_pos - original_actuator_pos) >= 0.02 ? 1.0 : 0.0;

      const double w_clearance = 1.0;
      const double w_manip = 0.1;
      const double w_joint = 0.5;
      const double w_limit = 0.2;
      const double w_actuator_move = 0.15;
      double score = w_clearance * clearance + w_manip * manipulability
          - w_joint * joint_dist + w_limit * limit_margin + w_centre * centre_bonus + w_actuator_move * actuator_moved;

        std::vector<double> joint_positions;
        seed_state->copyJointGroupPositions(jmg, joint_positions);
        scored.push_back({joint_positions, score});
      }
    }
  }

  if (scored.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid IK solution found (special logic, %zu actuator seeds, perturbs=%d)",
        actuator_seeds_spec.size(), perturbation_attempts);
    return {};
  }

  std::sort(scored.begin(), scored.end(),
      [](const ScoredSolution& a, const ScoredSolution& b) { return a.score > b.score; });

  std::vector<std::vector<double>> sorted_solutions;
  sorted_solutions.reserve(scored.size());
  for (const auto& s : scored)
    sorted_solutions.push_back(s.joints);

  RCLCPP_INFO(get_logger(), "Special logic: %zu IK solutions (best score=%.4f, worst=%.4f)",
      sorted_solutions.size(), scored.front().score, scored.back().score);
  return sorted_solutions;
}

void MotionControlNode::planToPoseCallback(
    const std::shared_ptr<arpa_control::srv::PlanToPose::Request> request,
    std::shared_ptr<arpa_control::srv::PlanToPose::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() START");
  const bool use_corridor = this->get_parameter("use_corridor_constraint").as_bool();

  if (m_use_depth)
  {
    bool reset_depth = resetDepthMap(2000);
    if(!reset_depth) {
      RCLCPP_ERROR(get_logger(), "Depth map failed to update. Abandoning move to pose");
      response->success = false;
      response->message = "Depth reset failed.";
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(get_logger(), "Updating depth map before planning");
    bool depth_update_success = updateDepthMap(2000);
    if (!depth_update_success)
    {
      RCLCPP_ERROR(get_logger(), "Depth map failed to update. Abandoning move to pose");
      response->success = false;
      response->message = "Depth update failed.";
      return;
    }
  }

  // Transform pose to the MoveIt planning frame (use current time for lookup to avoid sim/wall clock mismatch)
  std::string planning_frame = m_move_group->getPlanningFrame();
  geometry_msgs::msg::PoseStamped target_pose_for_tf = request->target_pose;
  target_pose_for_tf.header.stamp = this->now();
  geometry_msgs::msg::PoseStamped target_pose_in_planning_frame;

  try {
    target_pose_in_planning_frame = m_tf_buffer->transform(
      target_pose_for_tf, planning_frame, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform pose from '%s' to '%s': %s",
                 request->target_pose.header.frame_id.c_str(),
                 planning_frame.c_str(), ex.what());
    response->success = false;
    response->message = std::string("Transform failed: ") + ex.what();
    return;
  }

  RCLCPP_INFO(get_logger(), "Planning to target pose in '%s': x: %.3f, y: %.3f, z: %.3f",
              planning_frame.c_str(),
              target_pose_in_planning_frame.pose.position.x,
              target_pose_in_planning_frame.pose.position.y,
              target_pose_in_planning_frame.pose.position.z);

  // #region agent log
  {
    std::ostringstream o;
    o << "{\"hypothesisId\":\"H3,H5\",\"location\":\"motion_control:planToPoseCallback\",\"message\":\"plan_request\",\"data\":{\"use_cartesian\":"
      << (request->use_cartesian ? "true" : "false") << ",\"planning_frame\":\"" << planning_frame << "\"}";
    o << ",\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
    const char* p = std::getenv("DEBUG_LOG_PATH");
    std::string log_path = p ? p : "/root/ros2_ws/.cursor/debug.log";
    std::ofstream f(log_path, std::ios::app);
    if (f) f << o.str();
  }
  RCLCPP_INFO(get_logger(), "[DEBUG_H3] use_cartesian=%s planning_frame=%s",
              request->use_cartesian ? "true" : "false", planning_frame.c_str());
  // #endregion

  // Publish static transform for visualization
  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.header.stamp = now();
  static_transform.header.frame_id = planning_frame;
  static_transform.child_frame_id = "target_pose";
  static_transform.transform.translation.x = target_pose_in_planning_frame.pose.position.x;
  static_transform.transform.translation.y = target_pose_in_planning_frame.pose.position.y;
  static_transform.transform.translation.z = target_pose_in_planning_frame.pose.position.z;
  static_transform.transform.rotation = target_pose_in_planning_frame.pose.orientation;
  m_static_transform_broadcaster->sendTransform(static_transform);

  // Multi-objective (MOGA-style) IK seed selection: clearance, manipulability, joint distance, limit margin.
  if (m_special_logic && !request->use_cartesian) {
    RCLCPP_INFO(get_logger(), "Using special logic (multi-objective IK seed selection)");
    m_current_target_pose = target_pose_in_planning_frame.pose;
    m_move_group->clearPoseTargets();

    auto solutions = configureForPlanningSpecial(m_current_target_pose);
    if (solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solutions (special logic)";
      return;
    }

    for (size_t i = 0; i < solutions.size(); ++i) {
      m_move_group->setStartStateToCurrentState();
      m_move_group->setJointValueTarget(solutions[i]);

      auto plan_result = m_move_group->plan(m_current_plan);
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Planning succeeded (special logic) on IK solution %zu/%zu (%zu trajectory points)",
                    i + 1, solutions.size(), m_current_plan.trajectory_.joint_trajectory.points.size());
        m_goal_joint_values = solutions[i];

        auto goal_state = m_move_group->getCurrentState();
        goal_state->setJointGroupPositions(
            goal_state->getJointModelGroup(m_move_group->getName()), solutions[i]);
        goal_state->update();
        updateGoalMarker(goal_state);

        response->success = true;
        response->message = "Planning successful (special logic, solution " + std::to_string(i + 1) + "/" + std::to_string(solutions.size()) + ")";
        return;
      }

      RCLCPP_WARN(get_logger(), "Planning failed for IK solution %zu/%zu (special logic) (MoveItErrorCode: %d)",
                  i + 1, solutions.size(), plan_result.val);
    }

    response->success = false;
    response->message = "Planning failed for all " + std::to_string(solutions.size()) + " IK solutions (special logic)";
    RCLCPP_ERROR(get_logger(), "Planning failed for all %zu IK solutions (special logic)", solutions.size());
    return;
  }

  // [Checklist 1] When use_cartesian is true, use computeCartesianPath() for straight-line motion (no RRT).
  if (request->use_cartesian) {
    // Cartesian path planning: straight-line motion to target.
    // Uses full planning scene (gantry, pillars, battery, arm links) for collision checking - no self-collision
    // or gantry collision. Fix gantry (linear actuator) at current position so only the arm moves - straightforward
    // motion, no redundant solutions.
    RCLCPP_INFO(get_logger(), "Using Cartesian (straight-line) path planning with gantry fixed");

    m_move_group->setStartStateToCurrentState();

    // Constrain linear actuator to current position so IK uses only the arm (6 DOF) along the path
    const std::string gantry_joint = "linear_actuator_to_linear_actuator_plate_joint";
    moveit_msgs::msg::Constraints path_constraints;
    auto robot_state = m_move_group->getCurrentState();
    if (robot_state && robot_state->getRobotModel()->hasJointModel(gantry_joint)) {
      const double* pos = robot_state->getJointPositions(gantry_joint);
      if (pos) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = gantry_joint;
        jc.position = pos[0];
        jc.tolerance_above = 1e-6;
        jc.tolerance_below = 1e-6;
        jc.weight = 1.0;
        path_constraints.joint_constraints.push_back(jc);
        m_move_group->setPathConstraints(path_constraints);
        RCLCPP_INFO(get_logger(), "Gantry fixed at %.3f for Cartesian path", pos[0]);
      }
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose_in_planning_frame.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;   // 1cm step; coarser than 5mm so IK chain is less likely to abort
    const double jump_threshold = 1.5;  // Allow small joint-space jumps (0 can abort on achievable paths)
    double fraction = m_move_group->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    m_move_group->clearPathConstraints();

    // #region agent log
    {
      std::ostringstream o;
      o << "{\"hypothesisId\":\"H3,H4\",\"location\":\"motion_control:cartesian_result\",\"message\":\"cartesian_path_result\",\"data\":{\"fraction\":"
        << fraction << ",\"branch\":\"cartesian\"}";
      o << ",\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
      const char* lp = std::getenv("DEBUG_LOG_PATH");
      std::string log_path = lp ? lp : "/root/ros2_ws/.cursor/debug.log";
      std::ofstream f(log_path, std::ios::app);
      if (f) f << o.str();
    }
    RCLCPP_INFO(get_logger(), "[DEBUG_H4] branch=cartesian fraction=%.2f", fraction);
    // #endregion
    if (fraction >= 0.95) {
      m_current_plan.trajectory_ = trajectory;
      response->success = true;
      response->message = "Cartesian planning successful (fraction: " + std::to_string(fraction) + ")";
      RCLCPP_INFO(get_logger(), "Cartesian path computed: %.1f%% achieved", fraction * 100.0);
    } else {
      response->success = false;
      response->message = "Cartesian planning failed (only " + std::to_string(fraction * 100.0) + "% achieved)";
      RCLCPP_ERROR(get_logger(), "Cartesian path only achieved %.1f%%", fraction * 100.0);
    }
  } else if (use_corridor) {
    // #region agent log
    {
      std::ostringstream o;
      o << "{\"hypothesisId\":\"H3,H4\",\"location\":\"motion_control:rrt_branch\",\"message\":\"planning_branch\",\"data\":{\"branch\":\"rrt\"}";
      o << ",\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
      const char* lp = std::getenv("DEBUG_LOG_PATH");
      std::string log_path = lp ? lp : "/root/ros2_ws/.cursor/debug.log";
      std::ofstream f(log_path, std::ios::app);
      if (f) f << o.str();
    }
    RCLCPP_INFO(get_logger(), "[DEBUG_H4] branch=rrt");
    // #endregion
    m_move_group->setStartStateToCurrentState();
    clearCorridorMarker();

    // Hoist corridor geometry to block scope so fallbacks can reuse it
    const std::string ee_link = m_move_group->getEndEffectorLink();
    const double padding   = this->get_parameter("corridor_padding").as_double();
    const double cross_side = this->get_parameter("corridor_cross_section").as_double();
    const double cross_z    = this->get_parameter("corridor_cross_section_z").as_double();
    Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d end_pos(
      target_pose_in_planning_frame.pose.position.x,
      target_pose_in_planning_frame.pose.position.y,
      target_pose_in_planning_frame.pose.position.z);
    double seg_len = 0.0;

    auto robot_state = m_move_group->getCurrentState();
    if (robot_state) {
      start_pos = robot_state->getGlobalLinkTransform(ee_link).translation();
      seg_len = (end_pos - start_pos).norm();
      if (seg_len >= 1e-6) {
        setCorridorPathConstraints(
            start_pos, end_pos, padding, cross_side, cross_z,
            target_pose_in_planning_frame.pose.orientation,
            planning_frame, ee_link);
        RCLCPP_INFO(get_logger(), "RRT corridor: segment %.3f m, side %.3f m, z %.3f m",
            seg_len, 2.0 * cross_side, 2.0 * cross_z);
      }
    }

    auto solutions = configureForPlanning(target_pose_in_planning_frame.pose);
    if (solutions.empty()) {
      clearCorridorMarker();
      m_move_group->clearPathConstraints();
      response->success = false;
      response->message = "No valid IK solutions found";
      return;
    }

    // Re-rank IK solutions for corridor planning: prefer solutions where the gantry (joint index 0)
    // is closest to the estimated target actuator position (actuator_pos ≈ 1.0244 - target.x).
    // Cost-based ranking (joint distance from current) is wrong for corridor — "close to current"
    // doesn't mean "arm stays inside the box mid-motion". Gantry-near-target postures do.
    {
      const double corr_actuator_target =
          1.0244 - target_pose_in_planning_frame.pose.position.x;
      std::stable_sort(solutions.begin(), solutions.end(),
          [corr_actuator_target](const std::vector<double>& a, const std::vector<double>& b) {
            return std::abs(a[0] - corr_actuator_target) < std::abs(b[0] - corr_actuator_target);
          });
      RCLCPP_INFO(get_logger(), "Corridor re-ranked %zu IK solutions (actuator target est=%.3f)",
          solutions.size(), corr_actuator_target);
    }

    // Try every IK solution with whatever path constraints are currently set.
    // Returns the winning solution index, or -1 if all failed.
    auto tryAllSolutions = [&](const std::string& label) -> int {
      for (size_t i = 0; i < solutions.size(); ++i) {
        m_move_group->setStartStateToCurrentState();
        m_move_group->setJointValueTarget(solutions[i]);
        if (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(get_logger(), "Planning succeeded (%s) on IK solution %zu/%zu (%zu points)",
              label.c_str(), i + 1, solutions.size(),
              m_current_plan.trajectory_.joint_trajectory.points.size());
          return static_cast<int>(i);
        }
        RCLCPP_WARN(get_logger(), "Planning failed for IK solution %zu/%zu (%s)",
            i + 1, solutions.size(), label.c_str());
      }
      return -1;
    };

    // --- PASS 1: Normal (tight) corridor ---
    int winning_idx = tryAllSolutions("RRT corridor");

    // --- FALLBACK: Orientation-only (no position corridor) ---
    // Drops the box constraint entirely; only the tool-pointing-down orientation
    // constraint remains, giving RRT full joint-space freedom while still keeping
    // the end-effector orientation locked.
    if (winning_idx < 0) {
      RCLCPP_WARN(get_logger(), "Wide corridor failed — retrying with orientation-only fallback");
      m_move_group->clearPathConstraints();
      moveit_msgs::msg::OrientationConstraint ocm;
      ocm.link_name = ee_link;
      ocm.header.frame_id = planning_frame;
      ocm.orientation = target_pose_in_planning_frame.pose.orientation;
      // Looser than the corridor constraint (0.5 rad): trajectories near the workspace
      // boundary have waypoints that land on the tight-tolerance boundary and get rejected
      // by the trajectory validator even when the RRT finds a valid path. 0.8 rad gives
      // intermediate states room to pass validation; the goal orientation is still enforced
      // via the IK seed joint values.
      ocm.absolute_x_axis_tolerance = 0.8;
      ocm.absolute_y_axis_tolerance = 0.8;
      ocm.absolute_z_axis_tolerance = 2.0 * M_PI;
      ocm.weight = 1.0;
      moveit_msgs::msg::Constraints orientation_only;
      orientation_only.orientation_constraints.push_back(ocm);
      m_move_group->setPathConstraints(orientation_only);
      winning_idx = tryAllSolutions("RRT orientation-only fallback");
    }

    clearCorridorMarker();
    m_move_group->clearPathConstraints();

    if (winning_idx >= 0) {
      m_goal_joint_values = solutions[winning_idx];
      auto goal_state = m_move_group->getCurrentState();
      goal_state->setJointGroupPositions(
          goal_state->getJointModelGroup(m_move_group->getName()), solutions[winning_idx]);
      goal_state->update();
      updateGoalMarker(goal_state);
      response->success = true;
      response->message = "Planning successful (corridor branch, solution " +
          std::to_string(winning_idx + 1) + "/" + std::to_string(solutions.size()) + ")";
      RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
      return;
    }

    response->success = false;
    response->message = "Planning failed for all " + std::to_string(solutions.size()) +
        " IK solutions (all corridor fallbacks exhausted)";
    RCLCPP_ERROR(get_logger(), "Planning failed for all %zu IK solutions (all fallbacks)", solutions.size());
    RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
    return;
  } else {
    m_current_target_pose = target_pose_in_planning_frame.pose;

    m_move_group->clearPoseTargets();

    // Get IK solutions sorted by ascending cost
    auto solutions = configureForPlanning(m_current_target_pose);
    if (solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solutions found";
      return;
    }

    // Try planning with each solution until one succeeds
    for (size_t i = 0; i < solutions.size(); ++i) {
      m_move_group->setStartStateToCurrentState();
      m_move_group->setJointValueTarget(solutions[i]);

      auto plan_result = m_move_group->plan(m_current_plan);
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Planning succeeded on IK solution %zu/%zu (%zu trajectory points)",
                    i + 1, solutions.size(), m_current_plan.trajectory_.joint_trajectory.points.size());
        m_goal_joint_values = solutions[i];

        // Update goal marker
        auto goal_state = m_move_group->getCurrentState();
        goal_state->setJointGroupPositions(
            goal_state->getJointModelGroup(m_move_group->getName()), solutions[i]);
        goal_state->update();
        updateGoalMarker(goal_state);

        response->success = true;
        response->message = "Planning successful (solution " + std::to_string(i + 1) + "/" + std::to_string(solutions.size()) + ")";
        RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
        return;
      }

      RCLCPP_WARN(get_logger(), "Planning failed for IK solution %zu/%zu (MoveItErrorCode: %d)",
                  i + 1, solutions.size(), plan_result.val);
    }

    response->success = false;
    response->message = "Planning failed for all " + std::to_string(solutions.size()) + " IK solutions";
    RCLCPP_ERROR(get_logger(), "Planning failed for all %zu IK solutions", solutions.size());
    RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
  }
}


void MotionControlNode::executePlanCallback(
    const std::shared_ptr<arpa_control::srv::ExecutePlan::Request> request,
    std::shared_ptr<arpa_control::srv::ExecutePlan::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control executePlanCallback() START");

  size_t num_points = m_current_plan.trajectory_.joint_trajectory.points.size();
  RCLCPP_INFO(get_logger(), "Executing stored plan (%zu trajectory points)...", num_points);
  if (num_points == 0) {
    response->success = false;
    response->message = "No plan to execute (trajectory empty)";
    RCLCPP_ERROR(get_logger(), "No plan to execute: trajectory has 0 points");
    RCLCPP_INFO(get_logger(), "[TRACE] Motion Control executePlanCallback() END");
    return;
  }

  auto execute_result = m_move_group->execute(m_current_plan);
  if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    response->success = true;
    response->message = "Execution successful";
  }
  else
  {
    response->success = false;
    response->message = "Execution failed (MoveItErrorCode: " + std::to_string(execute_result.val) + ")";
    RCLCPP_ERROR(get_logger(), "Execution failed with MoveItErrorCode: %d (e.g. 1=FAILURE, 2=PLANNING_FAILED, 4=PREEMPTED, 5=CONTROL_FAILED)", execute_result.val);
  }
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control executePlanCallback() END");
}

void MotionControlNode::updateDepthCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control updateDepthCallback() START");
  bool result = updateDepthMap(10000);
  if(result) {
    response->message = "Depth map updated";
  } else {
    RCLCPP_ERROR(get_logger(), "Depth map update failed");
    response->message = "Depth map update failed";
  }
  response->success = result;
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control updateDepthCallback() END");
}
 
bool MotionControlNode::resetDepthMap(unsigned int timeout_ms)
{
  int count = 0;
  while (!m_depth_reset_client->wait_for_service(std::chrono::milliseconds(100)) && count < 100)
  {
    if (count % 10 == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting to connect to depth pointcloud service");
    }
    count++;
  }
  if (count == 100)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to depth pointcloud service. Depth not available.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto point_cloud_reset_result = m_depth_reset_client->async_send_request(request);
  while (rclcpp::ok() && (point_cloud_reset_result.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready))
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for Point Cloud response from PointCloud Accumulator.");
  }

  if (point_cloud_reset_result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive Point Cloud from PointCloud Accumulator within timeout.");
    return false;
  }

  return true;
}

bool MotionControlNode::updateDepthMap(unsigned int timeout_ms)
{
  RCLCPP_ERROR(get_logger(), "[TRACE] Motion Control updateDepthMap() START");
  using namespace std::chrono;

  auto start = steady_clock::now();
  auto deadline = start + std::chrono::milliseconds(timeout_ms);

  // ---- Wait for service ----
  while (!m_depth_client->wait_for_service(100ms)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for depth service.");
      return false;
    }
    if (steady_clock::now() >= deadline) {
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout waiting for depth service.");
      return false;
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for depth pointcloud service...");
  }

  // ---- Send async request ----
  auto request = std::make_shared<arpa_control::srv::GetPointCloud::Request>();
  auto future = m_depth_client->async_send_request(request);

  // ---- Wait for the response (executor thread will handle it) ----
  auto remaining = deadline - steady_clock::now();
  if (remaining <= std::chrono::milliseconds(0)) {
    RCLCPP_ERROR(this->get_logger(), "Timeout before waiting on response.");
    return false;
  }

  if (future.wait_for(remaining) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(),
                 "Timed out waiting for pointcloud response.");
    return false;
  }

  auto response = future.get();
  if (!response) {
    RCLCPP_ERROR(this->get_logger(), "Null pointcloud response.");
    return false;
  }

  auto cloud = response->cloud;

  if (cloud.data.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Received empty Point Cloud from PointCloud Accumulator.");
    return false;
  }

  // Convert PointCloud2 to OctoMap
  //TODO use parameter
  octomap::OcTree tree(0.03); // 5cm resolution
  octomap::Pointcloud octo_cloud;

  // Fill octomap pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x"), it_y(cloud, "y"), it_z(cloud, "z");
       it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
  {
    if (std::isfinite(*it_x) && std::isfinite(*it_y) && std::isfinite(*it_z))
      octo_cloud.push_back(*it_x, *it_y, *it_z);
  }

  tree.insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0));
  tree.updateInnerOccupancy();

  // Convert to ROS Octomap msg
  octomap_msgs::msg::Octomap octomap_msg;
  octomap_msgs::fullMapToMsg(tree, octomap_msg);
  octomap_msg.header.frame_id = "floor_link";  // IMPORTANT: must be TF-connected to planning frame
  octomap_msg.header.stamp = now();

  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.octomap.octomap = octomap_msg;
  if(!m_planning_scene_interface->applyPlanningScene(planning_scene_msg)) {
      throw std::runtime_error("Failed to apply planning scene");
  }
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control updateDepthMap() END");
  return true;
}

void MotionControlNode::stopMotionCallback(
    const std::shared_ptr<arpa_control::srv::StopMotion::Request>,
    std::shared_ptr<arpa_control::srv::StopMotion::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control stopMotionCallback() START");
  m_move_group->stop();
  response->success = true;
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control stopMotionCallback() END");
}

geometry_msgs::msg::PoseStamped MotionControlNode::poseToPlanningFrame(
    const geometry_msgs::msg::PoseStamped& pose_stamped)
{
  const std::string planning_frame = m_move_group->getPlanningFrame();

  if (pose_stamped.header.frame_id.empty() || pose_stamped.header.frame_id == planning_frame) {
    geometry_msgs::msg::PoseStamped result = pose_stamped;
    result.header.frame_id = planning_frame;
    return result;
  }

  geometry_msgs::msg::PoseStamped result;
  try {
    tf2::doTransform(pose_stamped, result,
        m_tf_buffer->lookupTransform(planning_frame, pose_stamped.header.frame_id, tf2::TimePointZero));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(get_logger(), "poseToPlanningFrame: TF lookup failed: %s", ex.what());
    return result;
  }
  return result;
}

double MotionControlNode::computePairwiseCost(
    const geometry_msgs::msg::PoseStamped& src_pose,
    const geometry_msgs::msg::PoseStamped& tgt_pose,
    const moveit::core::JointModelGroup* jmg,
    const std::string& ee_link)
{
  const int num_samples = 25;
  double min_cost = std::numeric_limits<double>::infinity();
  int src_ik_failures = 0;
  int tgt_ik_failures = 0;
  int valid_samples = 0;

  for (int i = 0; i < num_samples; ++i) {
    auto src_state = std::make_shared<moveit::core::RobotState>(m_move_group->getRobotModel());
    src_state->setToRandomPositions(jmg);
    src_state->update();
    if (!src_state->setFromIK(jmg, src_pose.pose, ee_link, 0.1)) {
      ++src_ik_failures;
      continue;
    }
    src_state->update();

    auto tgt_state = std::make_shared<moveit::core::RobotState>(*src_state);
    if (!tgt_state->setFromIK(jmg, tgt_pose.pose, ee_link, 0.1)) {
      ++tgt_ik_failures;
      continue;
    }
    tgt_state->update();

    std::vector<double> src_joints, tgt_joints;
    src_state->copyJointGroupPositions(jmg, src_joints);
    tgt_state->copyJointGroupPositions(jmg, tgt_joints);

    double cost = 0.0;
    for (size_t j = 0; j < src_joints.size() && j < m_joint_weights.size(); ++j) {
      double diff = tgt_joints[j] - src_joints[j];
      cost += m_joint_weights[j] * std::abs(diff);
    }

    ++valid_samples;
    if (cost < min_cost) {
      min_cost = cost;
    }
  }

  RCLCPP_DEBUG(get_logger(), "computePairwiseCost: %d/%d valid samples, src_ik_fail=%d, tgt_ik_fail=%d, min_cost=%.4f",
      valid_samples, num_samples, src_ik_failures, tgt_ik_failures, min_cost);

  if (valid_samples == 0) {
    RCLCPP_WARN(get_logger(), "computePairwiseCost: no valid IK solutions found (src_fail=%d, tgt_fail=%d)",
        src_ik_failures, tgt_ik_failures);
  }

  return min_cost;
}


void MotionControlNode::getPoseCostMatrixCallback(
    const std::shared_ptr<arpa_control::srv::GetPoseCostMatrix::Request> request,
    std::shared_ptr<arpa_control::srv::GetPoseCostMatrix::Response> response)
{
  const size_t N = request->poses.size();
  RCLCPP_INFO(get_logger(), "getPoseCostMatrix: received %zu poses", N);

  if (N == 0) {
    RCLCPP_WARN(get_logger(), "getPoseCostMatrix: empty poses array");
    response->success = false;
    response->message = "Empty poses array";
    response->size = 0;
    return;
  }

  const auto* jmg = m_move_group->getRobotModel()->getJointModelGroup(m_move_group->getName());
  const std::string ee_link = m_move_group->getEndEffectorLink();

  // Transform all poses to planning frame up front
  RCLCPP_INFO(get_logger(), "getPoseCostMatrix: transforming %zu poses to planning frame", N);
  std::vector<geometry_msgs::msg::PoseStamped> planning_poses(N);
  for (size_t i = 0; i < N; ++i) {
    planning_poses[i] = poseToPlanningFrame(request->poses[i]);
    if (planning_poses[i].header.frame_id.empty()) {
      RCLCPP_ERROR(get_logger(), "getPoseCostMatrix: TF lookup failed for pose %zu", i);
      response->success = false;
      response->message = "TF lookup failed for pose index " + std::to_string(i);
      response->size = 0;
      return;
    }
  }

  response->cost_matrix.resize(N * N, std::numeric_limits<double>::infinity());
  response->size = static_cast<uint32_t>(N);

  const size_t total_pairs = N * N;
  RCLCPP_INFO(get_logger(), "getPoseCostMatrix: computing %zu pairwise costs (parallel)", total_pairs);

  // Launch all off-diagonal pairs in parallel
  struct PairResult {
    size_t i, j;
    std::future<double> future;
  };
  std::vector<PairResult> futures;
  futures.reserve(N * (N - 1));

  for (size_t i = 0; i < N; ++i) {
    for (size_t j = 0; j < N; ++j) {
      if (i == j) continue;
      futures.push_back({i, j, std::async(std::launch::async,
          &MotionControlNode::computePairwiseCost, this,
          std::cref(planning_poses[i]), std::cref(planning_poses[j]), jmg, ee_link)});
    }
  }

  // Collect results
  size_t successful_entries = N;  // diagonal entries
  for (size_t i = 0; i < N; ++i) {
    response->cost_matrix[i * N + i] = 0.0;
  }
  for (auto& pr : futures) {
    double cost = pr.future.get();
    response->cost_matrix[pr.i * N + pr.j] = cost;
    if (!std::isinf(cost)) {
      ++successful_entries;
    }
  }
  RCLCPP_INFO(get_logger(), "getPoseCostMatrix: all pairs computed");

  response->success = (successful_entries > N);
  response->message = "Computed " + std::to_string(successful_entries) + "/" +
                      std::to_string(total_pairs) + " entries";
  RCLCPP_INFO(get_logger(), "getPoseCostMatrix: done — %zu/%zu successful entries, success=%s",
      successful_entries, total_pairs, response->success ? "true" : "false");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MotionControlNode>(options);
  node->init();
  node->initMoveGroup();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
