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
  //RVIZ uses 5s, 10 attempts, 0.1 vel scaling, 0.1 accel scaling
  m_move_group->setPlanningTime(5.0);//(5.0);
  m_move_group->setNumPlanningAttempts(10);//(10);
  m_move_group->setMaxVelocityScalingFactor(0.1);
  m_move_group->setMaxAccelerationScalingFactor(0.1);
  m_move_group->setGoalPositionTolerance(0.001);  // 1mm tolerance
  m_move_group->setGoalOrientationTolerance(0.001);  // ~0.057 degrees
  m_move_group->setGoalJointTolerance(0.001);  // 0.001 rad (~0.057 degrees) per joint

  m_move_group->allowReplanning(true);
  m_move_group->setReplanAttempts(3);
  m_move_group->setReplanDelay(0.1);  // seconds between replans

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
  //TODO
  //min elbow distance should be a param
  if (ee_distance < 0.650) {
    RCLCPP_WARN(get_logger(),
        "EE too close to linear actuator plate: %.3f m (min 0.60 m)", ee_distance);
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

  // RCLCPP_INFO(get_logger(), "Cost breakdown - Joint: %.4f, Proximity: %.4f (dist=%.3fm), Total: %.4f",
  //             joint_cost, proximity_penalty, actuator_wrist_distance, total_cost);

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

  std::vector<std::vector<double>> all_solutions;
  std::vector<double> all_costs;

  // Try IK with linear actuator offsets: 0, +0.1, -0.1, +0.2, -0.2, ... +/-1.0
  for (int step = 0; step <= 10; ++step) {
    std::vector<double> offsets;
    if (step == 0) {
      offsets.push_back(0.0);
    } else {
      offsets.push_back(step * 0.1);
      offsets.push_back(-step * 0.1);
    }

    for (double offset : offsets) {
      auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
      double shifted_pos = original_actuator_pos + offset;
      seed_state->setJointPositions(actuator_joint, &shifted_pos);
      seed_state->update();

      if (!seed_state->setFromIK(jmg, target_pose, ee_link, 0.1)) {
        RCLCPP_DEBUG(get_logger(), "IK failed for actuator offset %.2f", offset);
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

  if (all_solutions.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid IK solution found across actuator offsets +/- 1.0 m");
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

  struct ScoredSolution { std::vector<double> joints; double score; };
  std::vector<ScoredSolution> scored;

  for (int step = 0; step <= 10; ++step) {
    std::vector<double> offsets;
    if (step == 0)
      offsets.push_back(0.0);
    else {
      offsets.push_back(step * 0.1);
      offsets.push_back(-step * 0.1);
    }
    for (double offset : offsets) {
      auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
      double shifted_pos = original_actuator_pos + offset;
      seed_state->setJointPositions(actuator_joint, &shifted_pos);
      seed_state->update();
      if (!seed_state->setFromIK(jmg, target_pose, ee_link, 0.1)) continue;
      seed_state->update();

      if (getConfigurationCost(current_state, seed_state) == std::numeric_limits<double>::infinity())
        continue;

      double clearance = getMinClearance(seed_state);
      double manipulability = getManipulability(seed_state);
      double joint_dist = getWeightedJointDistance(current_state, seed_state);
      double limit_margin = getJointLimitMargin(seed_state);

      const double w_clearance = 1.0;
      const double w_manip = 0.1;
      const double w_joint = 0.5;
      const double w_limit = 0.2;
      double score = w_clearance * clearance + w_manip * manipulability
          - w_joint * joint_dist + w_limit * limit_margin;

      std::vector<double> joint_positions;
      seed_state->copyJointGroupPositions(jmg, joint_positions);
      scored.push_back({joint_positions, score});
    }
  }

  if (scored.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid IK solution found (special logic)");
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

    // Optional: constrain RRT to corridor (cuboid between current EE and target); off by default
    if (use_corridor) {
      auto robot_state = m_move_group->getCurrentState();
      if (robot_state) {
        const std::string ee_link = m_move_group->getEndEffectorLink();
        Eigen::Isometry3d ee_tf = robot_state->getGlobalLinkTransform(ee_link);
        Eigen::Vector3d start_pos = ee_tf.translation();
        Eigen::Vector3d end_pos(
          target_pose_in_planning_frame.pose.position.x,
          target_pose_in_planning_frame.pose.position.y,
          target_pose_in_planning_frame.pose.position.z);
        Eigen::Vector3d diff = end_pos - start_pos;
        double seg_len = diff.norm();
        if (seg_len >= 1e-6) {
          const double padding = this->get_parameter("corridor_padding").as_double();
          const double cross = this->get_parameter("corridor_cross_section").as_double();
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

          moveit_msgs::msg::Constraints path_constraints;
          path_constraints.position_constraints.push_back(pos_constraint);
          m_move_group->setPathConstraints(path_constraints);
          RCLCPP_INFO(get_logger(), "RRT corridor constraint: segment %.3f m, cross-section %.3f m", seg_len, 2.0 * cross);
        }
      }
    }

    auto solutions = configureForPlanning(target_pose_in_planning_frame.pose);
    if (solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solutions found";
      return;
    }

    // Try each IK solution with corridor constraint; set joint target so we plan to the requested pose
    for (size_t i = 0; i < solutions.size(); ++i) {
      m_move_group->setStartStateToCurrentState();
      m_move_group->setJointValueTarget(solutions[i]);

      auto plan_result = m_move_group->plan(m_current_plan);
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Planning succeeded (RRT corridor) on IK solution %zu/%zu (%zu trajectory points)",
                    i + 1, solutions.size(), m_current_plan.trajectory_.joint_trajectory.points.size());
        m_goal_joint_values = solutions[i];

        auto goal_state = m_move_group->getCurrentState();
        goal_state->setJointGroupPositions(
            goal_state->getJointModelGroup(m_move_group->getName()), solutions[i]);
        goal_state->update();
        updateGoalMarker(goal_state);

        m_move_group->clearPathConstraints();
        response->success = true;
        response->message = "Planning successful (RRT corridor, solution " + std::to_string(i + 1) + "/" + std::to_string(solutions.size()) + ")";
        RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
        return;
      }

      RCLCPP_WARN(get_logger(), "Planning failed for IK solution %zu/%zu (RRT corridor) (MoveItErrorCode: %d)",
                  i + 1, solutions.size(), plan_result.val);
    }

    // Fallback: retry without corridor constraint (path may have been invalid due to tight corridor)
    m_move_group->clearPathConstraints();
    RCLCPP_INFO(get_logger(), "RRT with corridor failed for all solutions, retrying without path constraints");
    solutions = configureForPlanning(target_pose_in_planning_frame.pose);
    if (solutions.empty()) {
      response->success = false;
      response->message = "No valid IK solutions found (after corridor fallback)";
      return;
    }

    for (size_t i = 0; i < solutions.size(); ++i) {
      m_move_group->setStartStateToCurrentState();
      m_move_group->setJointValueTarget(solutions[i]);

      auto plan_result = m_move_group->plan(m_current_plan);
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Planning succeeded (RRT no corridor fallback) on IK solution %zu/%zu (%zu trajectory points)",
                    i + 1, solutions.size(), m_current_plan.trajectory_.joint_trajectory.points.size());
        m_goal_joint_values = solutions[i];

        auto goal_state = m_move_group->getCurrentState();
        goal_state->setJointGroupPositions(
            goal_state->getJointModelGroup(m_move_group->getName()), solutions[i]);
        goal_state->update();
        updateGoalMarker(goal_state);

        response->success = true;
        response->message = "Planning successful (RRT fallback without corridor, solution " + std::to_string(i + 1) + "/" + std::to_string(solutions.size()) + ")";
        RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
        return;
      }

      RCLCPP_WARN(get_logger(), "Planning failed for IK solution %zu/%zu (RRT no corridor) (MoveItErrorCode: %d)",
                  i + 1, solutions.size(), plan_result.val);
    }

    response->success = false;
    response->message = "Planning failed for all " + std::to_string(solutions.size()) + " IK solutions (RRT with corridor and fallback)";
    RCLCPP_ERROR(get_logger(), "Planning failed for all %zu IK solutions (RRT corridor + fallback)", solutions.size());
    RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
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
    RCLCPP_ERROR(get_logger(), "Execution failed with MoveItErrorCode: %d", execute_result.val);
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
