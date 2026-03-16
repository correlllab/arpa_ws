#include "arpa_control/motion_control_node.hpp"
#include "arpa_control/ur16e_analytical_ik.hpp"
#include <rclcpp/exceptions.hpp>
#include <chrono>
#include <future>
#include <cmath>
#include <thread>
#include <atomic>
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
#include <set>
#include <random>
#include <moveit/robot_state/conversions.h>

MotionControlNode::MotionControlNode(rclcpp::NodeOptions options)
    : Node("motion_control_node", options)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Constructor Init");
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
      "/corridor_marker",
      rclcpp::QoS(1));

  m_use_depth = false; 
  //TODO verify octomap resolution is being used
  this->declare_parameter("octomap_resolution", 0.03);
  this->declare_parameter("arm_padding", 0.015);
  this->declare_parameter("cost_w_joint", 1.0);
  this->declare_parameter("cost_w_proximity", 1.0);
  this->declare_parameter("cost_w_area", 1.0);
  this->declare_parameter("use_analytical_ik", true);
  this->declare_parameter("kdl_random_restart_count", 1);
  this->declare_parameter("kdl_restart_timeout", 0.05);
  m_cost_w_joint = this->get_parameter("cost_w_joint").as_double();
  m_cost_w_proximity = this->get_parameter("cost_w_proximity").as_double();
  m_cost_w_area = this->get_parameter("cost_w_area").as_double();
  // planning_time is provided by launch (or can be set at runtime); do not declare here to avoid ParameterAlreadyDeclaredException when launch passes it
  m_arm_padding = this->get_parameter("arm_padding").as_double();
  m_arm_padding_links = {"forearm_link", "shoulder_link", "upper_arm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "tool_holder_link", "runner_link", "ratchet_extension_link"};
  for(auto link : m_arm_padding_links) {
    m_arm_padding_map[link] = m_arm_padding;
  }

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  // Create dedicated node for MoveGroupInterface — explicitly propagate use_sim_time
  // so the action client's clock matches the controller's clock (prevents execution hangs)
  m_move_group_node = rclcpp::Node::make_shared("move_group_interface_node", options);
  m_move_group_node->set_parameter(rclcpp::Parameter("use_sim_time",
      this->get_parameter("use_sim_time").as_bool()));

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
  // Velocity/accel scaling: 0.5 = 50% of max (faster execution; use 0.1 for cautious/slow)
  const double planning_time = this->get_parameter("planning_time").as_double();
  m_move_group->setPlanningTime(planning_time);
  m_move_group->setNumPlanningAttempts(10);  // Try up to 10 planning attempts per IK solution
  m_move_group->setMaxVelocityScalingFactor(0.5);
  m_move_group->setMaxAccelerationScalingFactor(0.5);
  m_move_group->setGoalPositionTolerance(0.001);  // 1mm tolerance
  m_move_group->setGoalOrientationTolerance(0.001);  // ~0.057 degrees
  m_move_group->setGoalJointTolerance(0.001);  // 0.001 rad (~0.057 degrees) per joint

  m_move_group->allowReplanning(true);
  m_move_group->setReplanAttempts(1);
  m_move_group->setReplanDelay(0.1);  // seconds between replans

  // Allow sensor updates during planning
  // m_move_group->allowLooking(true);

  // Other useful settings (commented out for reference)
  // m_move_group->setPoseReferenceFrame("world");   // frame for pose targets
  // m_move_group->setEndEffectorLink("tool0");           // which link to plan for
  // m_move_group->clearPathConstraints();                // remove constraints

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() END");
  dumpParams();
}

void MotionControlNode::dumpParams()
{

  //TODO come back and make sure everything we want printed  is printed
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

RawIKCost MotionControlNode::getRawConfigurationCost(
    const std::shared_ptr<moveit::core::RobotState>& current_state,
    const std::shared_ptr<moveit::core::RobotState>& target_state)
{
  RawIKCost result;

  const auto* jmg = target_state->getJointModelGroup(m_move_group->getName());

  if (!target_state->satisfiesBounds(jmg)) {
    RCLCPP_WARN(get_logger(), "Target state has joints out of valid range");
    return result;  // valid=false
  }

  const Eigen::Isometry3d& actuator_tf =
      target_state->getGlobalLinkTransform("linear_actuator_plate_link");
  const Eigen::Isometry3d& wrist_tf =
      target_state->getGlobalLinkTransform("wrist_3_link");
  const Eigen::Isometry3d& forearm_tf =
      target_state->getGlobalLinkTransform("forearm_link");

  double ee_distance = (actuator_tf.translation() - wrist_tf.translation()).norm();
  if (ee_distance < 0.650) {
    RCLCPP_WARN(get_logger(),
        "EE too close to linear actuator plate: %.3f m (min 0.65 m)", ee_distance);
    return result;  // valid=false
  }

  Eigen::Vector3d a = actuator_tf.translation();
  Eigen::Vector3d b = wrist_tf.translation();
  Eigen::Vector3d c = forearm_tf.translation();
  double triangle_area = 0.5 * (b - a).cross(c - a).norm();

  result.joint_distance = getWeightedJointDistance(current_state, target_state);
  result.proximity_penalty = 1.0 / ee_distance;
  result.area_penalty = 1.0 / triangle_area;
  result.valid = true;

  return result;
}

double MotionControlNode::getConfigurationCost(
    const std::shared_ptr<moveit::core::RobotState>& current_state,
    const std::shared_ptr<moveit::core::RobotState>& target_state)
{
  RawIKCost raw = getRawConfigurationCost(current_state, target_state);
  if (!raw.valid) {
    return std::numeric_limits<double>::infinity();
  }
  // Legacy combined cost (used by computePairwiseCost path)
  return (2.0 * raw.joint_distance) + (0.1 * raw.proximity_penalty) + (0.5 * raw.area_penalty);
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

std::vector<std::vector<double>> MotionControlNode::getJointConfigurations(geometry_msgs::msg::Pose target_pose)
{
  auto current_state = m_move_group->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state");
    return {};
  }

  const auto* jmg     = current_state->getJointModelGroup(m_move_group->getName());
  const std::string& ee_link = m_move_group->getEndEffectorLink();
  const std::string actuator_joint = "linear_actuator_to_linear_actuator_plate_joint";

  // Arm joint names in DH order (must match ur_manipulator group)
  const std::array<std::string, 6> arm_joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };

  double original_actuator_pos = *current_state->getJointPositions(actuator_joint);
  RCLCPP_INFO(get_logger(), "Current linear actuator position: %.3f m", original_actuator_pos);

  // Target pose as Eigen transform in world/planning frame
  Eigen::Isometry3d target_in_world = Eigen::Isometry3d::Identity();
  target_in_world.translation() = Eigen::Vector3d(
      target_pose.position.x, target_pose.position.y, target_pose.position.z);
  target_in_world.linear() = Eigen::Quaterniond(
      target_pose.orientation.w, target_pose.orientation.x,
      target_pose.orientation.y, target_pose.orientation.z).toRotationMatrix();

  // Re-read cost weights from params (allows runtime tuning via ros2 param set)
  m_cost_w_joint = this->get_parameter("cost_w_joint").as_double();
  m_cost_w_proximity = this->get_parameter("cost_w_proximity").as_double();
  m_cost_w_area = this->get_parameter("cost_w_area").as_double();
  RCLCPP_INFO(get_logger(), "Cost weights: joint=%.2f, proximity=%.2f, area=%.2f",
      m_cost_w_joint, m_cost_w_proximity, m_cost_w_area);

  // Pass 1: Collect all valid solutions and their raw costs
  std::vector<std::vector<double>> all_solutions;
  std::vector<RawIKCost> all_raw_costs;

  bool use_analytical = this->get_parameter("use_analytical_ik").as_bool();

  if (use_analytical) {
    // Sweep gantry positions at 0.1m steps and solve analytical IK at each
    std::set<int> tried_positions_mm;
    for (double offset : {0.0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3, -0.4, 0.4,
                          -0.5, 0.5, -0.6, 0.6, -0.7, 0.7, -0.8, 0.8,
                          -0.9, 0.9, -1.0, 1.0}) {
      double gantry_pos = std::clamp(original_actuator_pos + offset, 0.2, 1.9);
      int key_mm = static_cast<int>(gantry_pos * 1000);
      if (tried_positions_mm.count(key_mm)) continue;
      tried_positions_mm.insert(key_mm);

      auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
      seed_state->setJointPositions(actuator_joint, &gantry_pos);
      seed_state->update();
      Eigen::Isometry3d dh_frame0_in_world = seed_state->getGlobalLinkTransform("base_link_inertia");
      Eigen::Isometry3d target_in_dh0 = dh_frame0_in_world.inverse() * target_in_world;

      auto ik_solutions = ur16e_ik::solve(target_in_dh0);

      for (const auto& sol : ik_solutions) {
        for (int j = 0; j < 6; ++j) {
          seed_state->setJointPositions(arm_joint_names[j], &sol.joints[j]);
        }
        seed_state->update();

        if (!seed_state->satisfiesBounds(jmg)) continue;

        RawIKCost raw = getRawConfigurationCost(current_state, seed_state);
        if (!raw.valid) continue;

        std::vector<double> joint_positions;
        seed_state->copyJointGroupPositions(jmg, joint_positions);
        all_solutions.push_back(joint_positions);
        all_raw_costs.push_back(raw);
      }
    }
  }

  // KDL: single call from current state (when count=1) or N random restarts (count>1)
  {
    int kdl_restarts = this->get_parameter("kdl_random_restart_count").as_int();
    double kdl_timeout = this->get_parameter("kdl_restart_timeout").as_double();
    // k=0 uses current state as seed (preserves original single-call behavior when count=1)
    // k>0 uses random 7-DOF seeds to explore redundancy without structure
    double k0_timeout = (kdl_restarts == 1) ? 0.3 : kdl_timeout;
    for (int k = 0; k < kdl_restarts; ++k) {
      auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
      if (k > 0) {
        seed_state->setToRandomPositions(jmg);
        seed_state->update();
      }
      double timeout = (k == 0) ? k0_timeout : kdl_timeout;
      if (seed_state->setFromIK(jmg, target_pose, ee_link, timeout)) {
        seed_state->update();
        RawIKCost raw = getRawConfigurationCost(current_state, seed_state);
        if (raw.valid) {
          std::vector<double> joint_positions;
          seed_state->copyJointGroupPositions(jmg, joint_positions);
          all_solutions.push_back(joint_positions);
          all_raw_costs.push_back(raw);
        }
      }
    }
  }

  if (all_solutions.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid IK solution found (%s)",
        use_analytical ? "analytical + KDL" : "KDL only");
    return {};
  }

  // Pass 2: Normalize each cost component to [0,1] and combine with weights
  double min_joint = std::numeric_limits<double>::max();
  double max_joint = std::numeric_limits<double>::lowest();
  double min_prox  = std::numeric_limits<double>::max();
  double max_prox  = std::numeric_limits<double>::lowest();
  double min_area  = std::numeric_limits<double>::max();
  double max_area  = std::numeric_limits<double>::lowest();

  for (const auto& raw : all_raw_costs) {
    min_joint = std::min(min_joint, raw.joint_distance);
    max_joint = std::max(max_joint, raw.joint_distance);
    min_prox  = std::min(min_prox,  raw.proximity_penalty);
    max_prox  = std::max(max_prox,  raw.proximity_penalty);
    min_area  = std::min(min_area,  raw.area_penalty);
    max_area  = std::max(max_area,  raw.area_penalty);
  }

  double range_joint = max_joint - min_joint;
  double range_prox  = max_prox  - min_prox;
  double range_area  = max_area  - min_area;

  std::vector<double> all_costs(all_solutions.size());
  for (size_t i = 0; i < all_solutions.size(); ++i) {
    double norm_joint = (range_joint > 1e-12) ? (all_raw_costs[i].joint_distance - min_joint) / range_joint : 0.0;
    double norm_prox  = (range_prox  > 1e-12) ? (all_raw_costs[i].proximity_penalty - min_prox) / range_prox : 0.0;
    double norm_area  = (range_area  > 1e-12) ? (all_raw_costs[i].area_penalty - min_area) / range_area : 0.0;

    all_costs[i] = m_cost_w_joint * norm_joint + m_cost_w_proximity * norm_prox + m_cost_w_area * norm_area;
  }

  // Sort by normalized cost ascending (stable_sort preserves discovery order when costs are equal)
  std::vector<size_t> indices(all_solutions.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::stable_sort(indices.begin(), indices.end(),
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


double MotionControlNode::getManipulability(const std::shared_ptr<moveit::core::RobotState>& state)
{
  const auto* jmg = state->getJointModelGroup(m_move_group->getName());
  Eigen::MatrixXd jacobian = state->getJacobian(jmg);
  Eigen::MatrixXd JJt = jacobian * jacobian.transpose();
  double det = JJt.determinant();
  return (det > 0.0) ? std::sqrt(det) : 0.0;
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


void MotionControlNode::publishTargetTransform(geometry_msgs::msg::PoseStamped& target_pose){
  std::string planning_frame = m_move_group->getPlanningFrame();
  // Publish static transform for visualization
  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.header.stamp = now();
  static_transform.header.frame_id = planning_frame;
  static_transform.child_frame_id = "target_pose";
  static_transform.transform.translation.x = target_pose.pose.position.x;
  static_transform.transform.translation.y = target_pose.pose.position.y;
  static_transform.transform.translation.z = target_pose.pose.position.z;
  static_transform.transform.rotation = target_pose.pose.orientation;
  m_static_transform_broadcaster->sendTransform(static_transform);
}

bool MotionControlNode::setPathConstraints(geometry_msgs::msg::PoseStamped& target_pose){
  auto robot_state = m_move_group->getCurrentState();
  //TODO params should be member variables and not got each call
  const double padding = this->get_parameter("corridor_padding").as_double();
  const double cross = this->get_parameter("corridor_cross_section").as_double();
  const std::string planning_frame = m_move_group->getPlanningFrame();
  if (!robot_state){
    //TODO log something
    return false;
  }
  const std::string ee_link = m_move_group->getEndEffectorLink();
  Eigen::Isometry3d ee_tf = robot_state->getGlobalLinkTransform(ee_link);
  Eigen::Vector3d start_pos = ee_tf.translation();
  Eigen::Vector3d end_pos(
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);
  Eigen::Vector3d diff = end_pos - start_pos;
  double seg_len = diff.norm();
  if(seg_len < 1e-6) {
    RCLCPP_WARN(get_logger(), "Start and target poses are too close for corridor constraint (distance %.6f m)", seg_len);
    return false;
  }
  
  Eigen::Vector3d dir = diff / seg_len;
  double length_along = seg_len + 2.0 * padding;
  Eigen::Vector3d mid = start_pos + 0.5 * diff;
  Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir);
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
  box.dimensions[1] = 2.0 * cross;
  box.dimensions[2] = 2.0 * cross;

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

  // Publish corridor box for visualization in RViz
  visualization_msgs::msg::Marker corridor_marker;
  corridor_marker.header.frame_id = planning_frame;
  corridor_marker.header.stamp = now();
  corridor_marker.ns = "corridor";
  corridor_marker.id = 0;
  corridor_marker.type = visualization_msgs::msg::Marker::CUBE;
  corridor_marker.action = visualization_msgs::msg::Marker::ADD;
  corridor_marker.pose = box_pose;
  corridor_marker.scale.x = box.dimensions[0];
  corridor_marker.scale.y = box.dimensions[1];
  corridor_marker.scale.z = box.dimensions[2];
  corridor_marker.color.r = 0.0f;
  corridor_marker.color.g = 0.8f;
  corridor_marker.color.b = 1.0f;
  corridor_marker.color.a = 0.25f;
  m_corridor_marker_pub->publish(corridor_marker);

  moveit_msgs::msg::Constraints path_constraints;

  const bool constrain_position = this->get_parameter("constrain_corridor_position").as_bool();
  if (constrain_position) {
    path_constraints.position_constraints.push_back(pos_constraint);
  }

  const bool constrain_orientation = this->get_parameter("constrain_corridor_orientation").as_bool();
  if (constrain_orientation) {
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = planning_frame;
    oc.link_name = ee_link;

    // SLERP midpoint between start and goal orientations so both satisfy the constraint
    Eigen::Quaterniond q_start(ee_tf.rotation());
    Eigen::Quaterniond q_target(
        target_pose.pose.orientation.w,
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z);
    Eigen::Quaterniond q_mid = q_start.slerp(0.5, q_target);

    oc.orientation.x = q_mid.x();
    oc.orientation.y = q_mid.y();
    oc.orientation.z = q_mid.z();
    oc.orientation.w = q_mid.w();

    // Tolerance encompasses both start and goal with 0.2 rad margin
    double angular_distance = q_start.angularDistance(q_target);
    double tolerance = std::max(0.4, (angular_distance / 2.0) + 0.2);

    oc.absolute_x_axis_tolerance = tolerance;
    oc.absolute_y_axis_tolerance = tolerance;
    oc.absolute_z_axis_tolerance = tolerance;
    oc.weight = 1.0;
    path_constraints.orientation_constraints.push_back(oc);

    RCLCPP_INFO(get_logger(), "Orientation constraint: angular_distance=%.3f rad, tolerance=%.3f rad",
                angular_distance, tolerance);
  }

  m_move_group->setPathConstraints(path_constraints);
  RCLCPP_INFO(get_logger(), "RRT corridor constraint: segment %.3f m, cross-section %.3f m", seg_len, 2.0 * cross);
  return true;

}

void MotionControlNode::planToPoseCallback(
    const std::shared_ptr<arpa_control::srv::PlanToPose::Request> request,
    std::shared_ptr<arpa_control::srv::PlanToPose::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() START");
  response->num_ik_solutions = 0;
  response->selected_ik_solution_index = -1;
  // Apply current planning_time (allows runtime change via ros2 param set, e.g. per benchmark test case)
  const double planning_time = this->get_parameter("planning_time").as_double();
  m_move_group->setPlanningTime(planning_time);
  //TODO remove all move cartesian stuff
  //TODO set parameters in constructor?
  const bool use_corridor = this->get_parameter("use_corridor_constraint").as_bool();
  const bool constrain_orientation = this->get_parameter("constrain_corridor_orientation").as_bool();
  

  //TODO, put in a while loop with MAX_TRIES
  //TODO make a function
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
  geometry_msgs::msg::PoseStamped target_pose_in_planning_frame = poseToPlanningFrame(request->target_pose);

  RCLCPP_INFO(get_logger(), "\n\n[planToPoseCallback] Planning to target pose: x: %.3f, y: %.3f, z: %.3f",
              target_pose_in_planning_frame.pose.position.x,
              target_pose_in_planning_frame.pose.position.y,
              target_pose_in_planning_frame.pose.position.z);

  RCLCPP_INFO(get_logger(), "[planToPoseCallback] use_corridor=%s constrain_orientation=%s",
              std::to_string(use_corridor).c_str(),
              std::to_string(constrain_orientation).c_str());
  publishTargetTransform(target_pose_in_planning_frame);

  m_move_group->setStartStateToCurrentState();
  if (use_corridor) {
    setPathConstraints(target_pose_in_planning_frame);
  }
  
  m_current_target_pose = target_pose_in_planning_frame.pose;

  m_move_group->clearPoseTargets();

  // Get IK solutions sorted by ascending cost
  auto solutions = getJointConfigurations(m_current_target_pose);
  response->num_ik_solutions = static_cast<int32_t>(solutions.size());
  if (solutions.empty()) {
    response->success = false;
    response->manipulability_score = 0.0;
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
      goal_state->setJointGroupPositions(goal_state->getJointModelGroup(m_move_group->getName()), solutions[i]);
      goal_state->update();
      updateGoalMarker(goal_state);

      response->success = true;
      response->manipulability_score = getManipulability(goal_state);
      response->selected_ik_solution_index = static_cast<int32_t>(i + 1);
      response->message = "Planning successful (solution " + std::to_string(i + 1) + "/" + std::to_string(solutions.size()) + ")";
      RCLCPP_INFO(get_logger(), "Manipulability score: %.6f", response->manipulability_score);
      RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
      m_move_group->clearPathConstraints();                // remove constraints
      return;
    }

    RCLCPP_WARN(get_logger(), "Planning failed for IK solution %zu/%zu (MoveItErrorCode: %d)",
                i + 1, solutions.size(), plan_result.val);
  }

  response->success = false;
  response->manipulability_score = 0.0;
  response->message = "Planning failed for all " + std::to_string(solutions.size()) + " IK solutions";
  RCLCPP_ERROR(get_logger(), "Planning failed for all %zu IK solutions", solutions.size());
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
  m_move_group->clearPathConstraints();                // remove constraints


}


void MotionControlNode::executePlanCallback(
    const std::shared_ptr<arpa_control::srv::ExecutePlan::Request> request,
    std::shared_ptr<arpa_control::srv::ExecutePlan::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control executePlanCallback() START");

  // Compute timeout from trajectory duration + generous margin
  double traj_duration = 0.0;
  const auto& points = m_current_plan.trajectory_.joint_trajectory.points;
  if (!points.empty()) {
    traj_duration = rclcpp::Duration(points.back().time_from_start).seconds();
  }
  double timeout_s = traj_duration + 30.0;  // trajectory time + 30s margin

  // Run execute in a thread so we can enforce a timeout
  moveit::core::MoveItErrorCode execute_result;
  std::atomic<bool> finished{false};
  std::thread exec_thread([&]() {
    execute_result = m_move_group->execute(m_current_plan);
    finished.store(true);
  });

  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (!finished.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!finished.load()) {
    RCLCPP_ERROR(get_logger(), "Execution timed out after %.1fs (trajectory was %.1fs). Stopping robot.", timeout_s, traj_duration);
    m_move_group->stop();
    exec_thread.join();
    response->success = false;
    response->message = "Execution timed out after " + std::to_string(timeout_s) + "s";
  } else {
    exec_thread.join();
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = true;
      response->message = "Execution successful";
    } else {
      response->success = false;
      response->message = "Execution failed (MoveItErrorCode: " + std::to_string(execute_result.val) + ")";
      RCLCPP_ERROR(get_logger(), "Execution failed with MoveItErrorCode: %d", execute_result.val);
    }
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
    const std::string& ee_link,
    bool euclidean)
{
  if (euclidean) {
    const auto& sp = src_pose.pose.position;
    const auto& tp = tgt_pose.pose.position;
    double dx = sp.x - tp.x;
    double dy = sp.y - tp.y;
    double dz = sp.z - tp.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

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

    double cost = getWeightedJointDistance(src_state, tgt_state);

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


//BREAK OUT TSP INTO ITS OWN CLASS
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

  //TODO use only MAX_THREADS
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = 0; j < N; ++j) {
      if (i == j) continue;
      futures.push_back({i, j, std::async(std::launch::async,
          &MotionControlNode::computePairwiseCost, this,
          std::cref(planning_poses[i]), std::cref(planning_poses[j]), jmg, ee_link, request->euclidean)});
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
