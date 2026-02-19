#include "arpa_control/motion_control_node.hpp"
#include <chrono>
#include <future>
#include <cmath>
#include <limits>
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
  //TODO verify octomap resolution is being used
  this->declare_parameter("octomap_resolution", 0.03);
  this->declare_parameter("arm_padding", 0.015);
  m_arm_padding = this->get_parameter("arm_padding").as_double();
  m_arm_padding_links = {"forearm_link", "shoulder_link", "upper_arm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "tool_holder_link", "tool_center_link", "tool_head_link"};
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

  m_move_group->startStateMonitor(1.0);
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

bool MotionControlNode::configureForPlanning(geometry_msgs::msg::Pose target_pose)
{
  auto current_state = m_move_group->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state");
    return false;
  }

  const auto* jmg = current_state->getJointModelGroup(m_move_group->getName());
  const std::string& ee_link = m_move_group->getEndEffectorLink();
  const std::string actuator_joint = "linear_actuator_to_linear_actuator_plate_joint";

  // Get the original linear actuator position
  double original_actuator_pos = *current_state->getJointPositions(actuator_joint);

  RCLCPP_INFO(get_logger(), "Current linear actuator position: %.3f m", original_actuator_pos);

  double best_cost = std::numeric_limits<double>::infinity();
  std::shared_ptr<moveit::core::RobotState> best_state;
  double best_actuator_pos = original_actuator_pos;

  // Store all successful IK solutions
  std::vector<std::vector<double>> all_solutions;  // Each entry is joint positions for one solution
  std::vector<double> all_costs;                   // Corresponding costs
  std::vector<double> all_actuator_positions;      // Corresponding actuator positions

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
      // Reset seed state and apply actuator offset
      auto seed_state = std::make_shared<moveit::core::RobotState>(*current_state);
      double shifted_pos = original_actuator_pos + offset;
      seed_state->setJointPositions(actuator_joint, &shifted_pos);

      // Add small random perturbations to arm joints to explore different local minima
      std::vector<double> joint_values;
      seed_state->copyJointGroupPositions(jmg, joint_values);

      for (size_t i = 1; i < joint_values.size(); ++i) {  // Skip actuator (i=0)
        joint_values[i] += m_arm_noise_dist(m_rng);
      }
      seed_state->setJointGroupPositions(jmg, joint_values);
      seed_state->update();

      // RCLCPP_INFO(get_logger(), "Attempting IK for offset %+.2f (actuator seed: %.3f m) with random arm perturbations",
      //             offset, shifted_pos);

      //TODO if some param, visualize
      // Visualize the seed state before IK
      // updateGoalMarker(seed_state);
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      bool ik_ok = seed_state->setFromIK(jmg, target_pose, ee_link, 0.1);

      if (!ik_ok) {
        RCLCPP_DEBUG(get_logger(), "IK failed for actuator offset %.2f", offset);
        continue;
      }

      // Get the actuator position that IK chose
      double ik_actuator_pos = *seed_state->getJointPositions(actuator_joint);

      seed_state->update();

      //TODO if some param, visualize
      // updateGoalMarker(seed_state);
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      double cost = getConfigurationCost(current_state, seed_state);

      // Store this valid solution
      std::vector<double> joint_positions;
      seed_state->copyJointGroupPositions(jmg, joint_positions);
      all_solutions.push_back(joint_positions);
      all_costs.push_back(cost);
      all_actuator_positions.push_back(ik_actuator_pos);

      // RCLCPP_INFO(get_logger(), "Actuator offset %+.2f (seed: %.3f, ik: %.3f) -> cost: %.4f [solution #%zu]",
      //             offset, shifted_pos, ik_actuator_pos, cost, all_solutions.size());

      // Sleep to allow visualization of this candidate
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      if (cost < best_cost) {
        best_cost = cost;
        best_state = std::make_shared<moveit::core::RobotState>(*seed_state);
        best_actuator_pos = ik_actuator_pos;
      }
    }
  }

  // RCLCPP_INFO(get_logger(), "Found %zu total IK solutions", all_solutions.size());

  // Calculate max difference for each joint across all solutions
  if (!all_solutions.empty()) {
    const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
    std::vector<double> min_vals(joint_names.size(), std::numeric_limits<double>::max());
    std::vector<double> max_vals(joint_names.size(), -std::numeric_limits<double>::max());

    for (const auto& solution : all_solutions) {
      for (size_t j = 0; j < solution.size(); ++j) {
        min_vals[j] = std::min(min_vals[j], solution[j]);
        max_vals[j] = std::max(max_vals[j], solution[j]);
      }
    }

    RCLCPP_INFO(get_logger(), "Joint variation across all solutions:");
    for (size_t j = 0; j < joint_names.size(); ++j) {
      double range = max_vals[j] - min_vals[j];
      RCLCPP_INFO(get_logger(), "  %s: range=%.4f rad (min=%.4f, max=%.4f)",
                  joint_names[j].c_str(), range, min_vals[j], max_vals[j]);
    }
  }

  if (best_state) {
    double actuator_movement = best_actuator_pos - original_actuator_pos;
    RCLCPP_INFO(get_logger(), "\n\nBest configuration: cost=%.4f, actuator_pos=%.3f m, movement=%+.3f m",
                best_cost, best_actuator_pos, actuator_movement);

    // Log the best IK joint values before setting them
    const auto* jmg_final = best_state->getJointModelGroup(m_move_group->getName());
    std::vector<double> best_joint_values;
    best_state->copyJointGroupPositions(jmg_final, best_joint_values);
    std::string best_joints_str = "Best state joint angles: ";
    for (size_t i = 0; i < best_joint_values.size(); ++i) {
      best_joints_str += std::to_string(best_joint_values[i]) + " ";
    }
    best_joints_str += "\n";
    RCLCPP_INFO(get_logger(), "%s", best_joints_str.c_str());

    // Store the goal joint values for comparison after execution
    m_goal_joint_values = best_joint_values;

    // Try setting joint target with explicit vector instead of RobotState
    // This might be more reliable for ensuring MoveIt uses joint-space planning
    m_move_group->setJointValueTarget(best_joint_values);

    // Update goal marker with final best configuration
    updateGoalMarker(best_state);

    return true;
  }

  RCLCPP_ERROR(get_logger(), "No valid IK solution found across actuator offsets +/- 1.0 m");
  return false;
}

void MotionControlNode::planToPoseCallback(
    const std::shared_ptr<arpa_control::srv::PlanToPose::Request> request,
    std::shared_ptr<arpa_control::srv::PlanToPose::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() START");

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

  // Transform pose to the MoveIt planning frame
  std::string planning_frame = m_move_group->getPlanningFrame();
  geometry_msgs::msg::PoseStamped target_pose_in_planning_frame;

  try {
    target_pose_in_planning_frame = m_tf_buffer->transform(
      request->target_pose, planning_frame, tf2::durationFromSec(1.0));
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

  m_current_target_pose = target_pose_in_planning_frame.pose;

  // Set start state first
  m_move_group->setStartStateToCurrentState();

  // Clear any existing pose targets to ensure we use joint values
  m_move_group->clearPoseTargets();

  // Find best IK solution and set as joint value target
  if (!configureForPlanning(m_current_target_pose)) {
    response->success = false;
    response->message = "Failed to configure for planning";
    return;
  }


  auto plan_result = m_move_group->plan(m_current_plan);
  if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
  {
    response->success = true;
    response->message = "Planning successful";
    RCLCPP_INFO(get_logger(), "Planning succeeded (%zu trajectory points)",
                m_current_plan.trajectory_.joint_trajectory.points.size());

  }
  else
  {
    response->success = false;
    response->message = "Planning failed (MoveItErrorCode: " + std::to_string(plan_result.val) + ")";
    RCLCPP_ERROR(get_logger(), "Planning failed with MoveItErrorCode: %d", plan_result.val);
  }
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
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
