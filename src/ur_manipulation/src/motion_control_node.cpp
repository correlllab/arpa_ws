#include "ur_manipulation/motion_control_node.hpp"
#include <chrono>
#include <future>
#include <cmath>

MotionControlNode::MotionControlNode(rclcpp::NodeOptions options)
    : Node("motion_control_node", options)
{
  RCLCPP_ERROR(get_logger(), "[TRACE] Constructor START");
  // Load robot description (for local IK solving)
  this->declare_parameter("robot_description", "");
  this->declare_parameter("robot_description_semantic", "");
  RCLCPP_ERROR(get_logger(), "[TRACE] Parameters declared");

  m_plan_to_pose_service = this->create_service<ur_manipulation::srv::PlanToPose>(
      "plan_to_pose",
      std::bind(&MotionControlNode::planToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  m_plan_to_joint_service = this->create_service<ur_manipulation::srv::PlanToJoint>(
      "plan_to_joint",
      std::bind(&MotionControlNode::planToJointCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_execute_plan_service = this->create_service<ur_manipulation::srv::ExecutePlan>(
      "execute_plan",
      std::bind(&MotionControlNode::executePlanCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_stop_motion_service = this->create_service<ur_manipulation::srv::StopMotion>(
      "stop_motion",
      std::bind(&MotionControlNode::stopMotionCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_update_depth_service = this->create_service<std_srvs::srv::Trigger>(
      "update_depth",
      std::bind(&MotionControlNode::updateDepthCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_depth_reset_client = this->create_client<std_srvs::srv::Trigger>("arm_pointcloud");


  m_depth_client_group =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Attach the client to that callback group
  m_depth_client = this->create_client<ur_manipulation::srv::GetPointCloud>(
    "/pointcloud_accumulator/get_arm_pointcloud",
    rmw_qos_profile_services_default,
      m_depth_client_group);
  
  // Static TF Broacaster 
  m_static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  m_use_depth = false;
  this->declare_parameter("octomap_resolution", 0.03);
  this->declare_parameter("arm_padding", 0.015);
  m_arm_padding = this->get_parameter("arm_padding").as_double();
  m_arm_padding_links = {"forearm_link", "shoulder_link", "upper_arm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "tool_holder_link", "tool_center_link", "tool_head_link"};
  for(auto link : m_arm_padding_links) {
    m_arm_padding_map[link] = m_arm_padding;
  }

  RCLCPP_ERROR(get_logger(), "[TRACE] MoveGroupInterface created");
  // m_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(m_plan_spin_node, "robot_description");
  RCLCPP_ERROR(get_logger(), "[TRACE] Constructor END");
}

void MotionControlNode::init()
{
  RCLCPP_ERROR(get_logger(), "[TRACE] init() START");
  RCLCPP_INFO(get_logger(), "Motion Control Node initialized and ready.");

  // if (!m_planning_scene_monitor->getPlanningScene())
  // {
    //   RCLCPP_ERROR(get_logger(), "Planning scene not configured. Motion Control Constucter FAILED.");
    //   return;
    // }

  // m_planning_scene_monitor->requestPlanningSceneState();
  // m_planning_scene_monitor->startSceneMonitor();
  // m_planning_scene_monitor->startStateMonitor();
  // m_planning_scene_monitor->startWorldGeometryMonitor();
  // m_planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "/planning_scene_handler/planning_scene");

  RCLCPP_ERROR(get_logger(), "[TRACE] Creating PlanningSceneInterface");
  m_planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "ur_manipulator");
    RCLCPP_ERROR(get_logger(), "[TRACE] init() END");
}

void MotionControlNode::initMoveGroup()
{
  RCLCPP_ERROR(get_logger(), "[TRACE] initMoveGroup() START");

  RCLCPP_ERROR(get_logger(), "[TRACE] Starting state monitor");
  m_move_group->startStateMonitor(1.0);
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting planner ID");
  m_move_group->setPlannerId("RRTConnectkConfigDefault");
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting planning pipeline ID");
  m_move_group->setPlanningPipelineId("move_group");
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting planner ID (second call)");
  m_move_group->setPlannerId("ur_manipulator");

  // Set goal position tolerance (meters)
  m_move_group->setGoalPositionTolerance(0.001);  // 1mm instead of default ~1cm

  // Set goal orientation tolerance (radians)
  m_move_group->setGoalOrientationTolerance(0.01);  // ~0.57 degrees

  // Set goal joint tolerance (radians)
  m_move_group->setGoalJointTolerance(0.001);  // Very tight

  // Create a one-shot timer to check when state is ready
  RCLCPP_ERROR(get_logger(), "[TRACE] Creating wall timer");
  m_init_timer = this->create_wall_timer(
      std::chrono::milliseconds(2500),
      std::bind(&MotionControlNode::checkRobotStateReady, this)
  );

  RCLCPP_INFO(get_logger(), "Motion Control Node initialization started. Waiting for robot state...");
  RCLCPP_ERROR(get_logger(), "[TRACE] initMoveGroup() END");
}

void MotionControlNode::checkRobotStateReady()
{
  RCLCPP_ERROR(get_logger(), "[TRACE] checkRobotStateReady() START");
  // Check if we have a current state
  auto current_state = m_move_group->getCurrentState(0.0001);  // Short timeout
  RCLCPP_ERROR(get_logger(), "[TRACE] getCurrentState() returned");
  RCLCPP_INFO(get_logger(), "Motion Control Node initialization started. Waiting for robot state...");

  if (current_state) {
    RCLCPP_INFO(this->get_logger(), "Robot state received! Motion Control Node fully ready.");
    m_robot_state_ready = true;
    // m_init_timer->cancel();  // Stop the timer
    // m_init_timer.reset();
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Still waiting for robot state...");
  }
  RCLCPP_ERROR(get_logger(), "[TRACE] checkRobotStateReady() END");
}

void MotionControlNode::planToPoseCallback(
    const std::shared_ptr<ur_manipulation::srv::PlanToPose::Request> request,
    std::shared_ptr<ur_manipulation::srv::PlanToPose::Response> response)
{
  RCLCPP_ERROR(get_logger(), "[TRACE] planToPoseCallback() START");
  RCLCPP_INFO(get_logger(), "Received plan_to_pose request");
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting start state to current");
  m_move_group->setStartStateToCurrentState();
  RCLCPP_ERROR(get_logger(), "[TRACE] Start state set"); 
  // First, request a lidar scan and populate internal perception before planning.
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
    else
    {
      RCLCPP_WARN(get_logger(), "Timed out waiting for lidar scan, proceeding without perception update");
    }
  }

  RCLCPP_INFO(get_logger(), "Planning to target pose: x: %.2f, y: %.2f, z: %.2f",
              request->target_pose.pose.position.x,
              request->target_pose.pose.position.y,
              request->target_pose.pose.position.z);

  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.header.stamp = now();
  static_transform.header.frame_id = "world";
  static_transform.child_frame_id = "target_pose";
  static_transform.transform.translation.x = request->target_pose.pose.position.x;
  static_transform.transform.translation.y = request->target_pose.pose.position.y;
  static_transform.transform.translation.z = request->target_pose.pose.position.z;
  static_transform.transform.rotation = request->target_pose.pose.orientation;
  m_static_transform_broadcaster->sendTransform(static_transform);


  // 1. Get current joint values as seed
  // auto current_state = m_move_group->getCurrentState();  // RobotStatePtr
  // current_state->update();
  // m_move_group->setStartState(*current_state);
  // std::vector<double> joint_values;
  // const auto& joint_model_group = m_move_group->getCurrentState()->getJointModelGroup("ur_manipulator");
  // m_move_group->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);

  // // 2. Build joint constraints
  // moveit_msgs::msg::Constraints constraints;
  // constraints.name = "keep_wrist_sane";

  // auto make_joint_constraint = [](const std::string &name, double position,
  //                                 double tolerance_above, double tolerance_below, double weight = 1.0) {
  //   moveit_msgs::msg::JointConstraint jc;
  //   jc.joint_name       = name;
  //   jc.position         = position;
  //   jc.tolerance_above  = tolerance_above;
  //   jc.tolerance_below  = tolerance_below;
  //   jc.weight           = weight;
  //   return jc;
  // };

  // // Assume UR joint names like these; adjust to your actual names
  // // e.g., "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", ...
  // constraints.joint_constraints.push_back(
  //     make_joint_constraint("elbow_joint", joint_values[2], 0.7, 0.7));    // ±0.7 rad
  // constraints.joint_constraints.push_back(
  //     make_joint_constraint("wrist_1_joint", joint_values[3], 0.7, 0.7));
  // constraints.joint_constraints.push_back(
  //     make_joint_constraint("wrist_2_joint", joint_values[4], 0.7, 0.7));
  // constraints.joint_constraints.push_back(
  //     make_joint_constraint("wrist_3_joint", joint_values[5], 0.7, 0.7));

  // // 3. Apply as path constraints
  // m_move_group->setPathConstraints(constraints);

  // m_move_group->setWorkspace(-1.0, -1.0, -1.0,
  //                          +1.0, +1.0, +1.0);

  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision.");
  // auto state = m_move_group->getCurrentState(1.0);
  // // m_move_group->setStartState(*m_move_group->getCurrentState(1.0));
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision2.");
  // auto current_pose = m_move_group->getCurrentPose();
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23.");
  // moveit_msgs::msg::PositionConstraint box_constraint;
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision234.");
  // box_constraint.header.frame_id = m_move_group->getPoseReferenceFrame();
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision2345.");
  // box_constraint.link_name = m_move_group->getEndEffectorLink();
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23456.");
  // shape_msgs::msg::SolidPrimitive box;
  // box.type = shape_msgs::msg::SolidPrimitive::BOX;
  // box.dimensions = { 0.5, 0.5, 0.5 };
  // box_constraint.constraint_region.primitives.emplace_back(box);
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision234567.");
  
  // geometry_msgs::msg::Pose box_pose;
  // box_pose.position.x = current_pose.pose.position.x;
  // box_pose.position.y = current_pose.pose.position.y;
  // box_pose.position.z = current_pose.pose.position.z;
  // box_pose.orientation.x = current_pose.pose.orientation.x;
  // box_pose.orientation.y = current_pose.pose.orientation.y;
  // box_pose.orientation.z = current_pose.pose.orientation.z;
  // box_pose.orientation.w = current_pose.pose.orientation.w;
  // box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision2345678.");
  // box_constraint.weight = 1.0;
  // moveit_msgs::msg::Constraints box_constraints;
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23456789.");
  // box_constraints.position_constraints.emplace_back(box_constraint);
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23456789a.");
  // m_move_group->setPathConstraints(box_constraints);
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23456789ab.");

  // Option 1: Remove position constraints entirely
  // m_move_group->clearPathConstraints();

  // // Option 2: Set correct constraint for current position
  // moveit_msgs::msg::Constraints constraints;
  // moveit_msgs::msg::PositionConstraint position_constraint;

  // position_constraint.link_name = "tool_holder_link";
  // position_constraint.header.frame_id = "base_link";  // or your base frame

  // // Create a large box around current position instead of (0,0,0)
  // shape_msgs::msg::SolidPrimitive primitive;
  // primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 2.0;
  // primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 2.0;
  // primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 2.0;

  // // Get current pose of the end effector
  // geometry_msgs::msg::PoseStamped current_pose = m_move_group->getCurrentPose("tool_holder_link");

  // position_constraint.constraint_region.primitives.push_back(primitive);
  // position_constraint.constraint_region.primitive_poses.push_back(current_pose.pose);
  // position_constraint.weight = 1.0;

  // constraints.position_constraints.push_back(position_constraint);
  // m_move_group->setPathConstraints(constraints);
  
  // m_move_group->setPlanningFrame("base_link");
  m_move_group->setPlanningTime(10.0);
  m_move_group->setNumPlanningAttempts(5);


  // // Get the robot model and current state
  // const moveit::core::JointModelGroup* joint_model_group = 
  //     m_move_group->getRobotModel()->getJointModelGroup("ur_manipulator");
  // moveit::core::RobotStatePtr current_state = m_move_group->getCurrentState(10.0);
  // if (!current_state) {
  //     RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "Failed to get current state");
  //     return ;
  // }


  // // Solve IK for the target pose
  // const std::string& end_effector_link = "tool0";  // or move_group.getEndEffectorLink()
  
  // bool found_ik = current_state->setFromIK(
  //     joint_model_group,
  //     request->target_pose.pose,
  //     end_effector_link,
  //     10.0,  // timeout in seconds
  //     moveit::core::GroupStateValidityCallbackFn()  // no collision check during IK
  // );


  // if (!found_ik) {
  //     RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "IK solution not found");
  //     return ;
  // }
  
  // // Get the joint values from the IK solution
  // std::vector<double> joint_values;
  // current_state->copyJointGroupPositions(joint_model_group, joint_values);
  
  // // Log the solution
  // RCLCPP_INFO(rclcpp::get_logger("motion_control"), "IK Solution found:");
  // const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
  // for (size_t i = 0; i < joint_names.size(); i++) {
  //     RCLCPP_INFO(rclcpp::get_logger("motion_control"), "  %s: %.4f", 
  //                 joint_names[i].c_str(), joint_values[i]);
  // }
    

  // // Now plan in joint space
  // m_move_group->setJointValueTarget(joint_values);
    
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23456789abc.");
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting pose target");
  m_move_group->setPoseTarget(request->target_pose.pose, "tool0");

  RCLCPP_ERROR(get_logger(), "[TRACE] Calling plan()");
  bool success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_ERROR(get_logger(), "[TRACE] plan() returned");

  if (success)
  {
    response->success = (success == moveit::core::MoveItErrorCode::SUCCESS);
    response->message = response->success ? "Planning successful" : "Planning failed";
  }
  else
  {
    response->success = false;
    response->message = "Planning failed";
  }
}


void MotionControlNode::planToJointCallback(
    const std::shared_ptr<ur_manipulation::srv::PlanToJoint::Request> request,
    std::shared_ptr<ur_manipulation::srv::PlanToJoint::Response> response)
{
  RCLCPP_ERROR(get_logger(), "[TRACE] planToJointCallback() START");
  RCLCPP_INFO(get_logger(), "Received plan_to_joint request");
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting start state to current");
  m_move_group->setStartStateToCurrentState();
  RCLCPP_ERROR(get_logger(), "[TRACE] Start state set"); 
  // First, request a lidar scan and populate internal perception before planning.
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
    else
    {
      RCLCPP_WARN(get_logger(), "Timed out waiting for lidar scan, proceeding without perception update");
    }
  }

  RCLCPP_INFO(get_logger(), "Planning to target joint: j1: %.2f, j2: %.2f, j3: %.2f, j4: %.2f, j5: %.2f, j6: %.2f",
              request->joint1,
              request->joint2,
              request->joint3,
              request->joint4,
              request->joint5,
              request->joint6);

  // 1. Get current joint values as seed
  // auto current_state = m_move_group->getCurrentState();  // RobotStatePtr
  // current_state->update();
  // m_move_group->setStartState(*current_state);
  // std::vector<double> joint_values;
  // const auto& joint_model_group = m_move_group->getCurrentState()->getJointModelGroup("ur_manipulator");
  // m_move_group->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);

  // // Get the robot model and current state
  // const moveit::core::JointModelGroup* joint_model_group = 
  //     m_move_group->getRobotModel()->getJointModelGroup("ur_manipulator");
  // moveit::core::RobotStatePtr current_state = m_move_group->getCurrentState(10.0);
  // if (!current_state) {
  //     RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "Failed to get current state");
  //     return ;
  // }

  // // Solve IK for the target pose
  // const std::string& end_effector_link = "tool0";  // or move_group.getEndEffectorLink()
  
  // bool found_ik = current_state->setFromIK(
  //     joint_model_group,
  //     request->target_pose.pose,
  //     end_effector_link,
  //     10.0,  // timeout in seconds
  //     moveit::core::GroupStateValidityCallbackFn()  // no collision check during IK
  // );


  // if (!found_ik) {
  //     RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "IK solution not found");
  //     return ;
  // }
  
  // // Get the joint values from the IK solution
  std::vector<double> joint_values;
  // current_state->copyJointGroupPositions(joint_model_group, joint_values);
  joint_values.push_back(request->joint1);
  joint_values.push_back(request->joint2);
  joint_values.push_back(request->joint3);
  joint_values.push_back(request->joint4);
  joint_values.push_back(request->joint5);
  joint_values.push_back(request->joint6);
  
  // // Log the solution
  // RCLCPP_INFO(rclcpp::get_logger("motion_control"), "IK Solution found:");
  // const std::vector<std::string>& joint_names = joint_model_group->getActiveJointModelNames();
  // for (size_t i = 0; i < joint_names.size(); i++) {
  //     RCLCPP_INFO(rclcpp::get_logger("motion_control"), "  %s: %.4f", 
  //                 joint_names[i].c_str(), joint_values[i]);
  // }
    

  // // Now plan in joint space
  m_move_group->setJointValueTarget(joint_values);
    
  // RCLCPP_INFO(get_logger(), "Planning with path constraints to avoid arm self-collision23456789abc.");
  // RCLCPP_ERROR(get_logger(), "[TRACE] Setting pose target");
  // m_move_group->setPoseTarget(request->target_pose.pose, "tool0");

  RCLCPP_ERROR(get_logger(), "[TRACE] Calling plan()");
  bool success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_ERROR(get_logger(), "[TRACE] plan() returned");

  if (success)
  {
    response->success = (success == moveit::core::MoveItErrorCode::SUCCESS);
    response->message = response->success ? "Planning successful" : "Planning failed";
  }
  else
  {
    response->success = false;
    response->message = "Planning failed";
  }
}

void MotionControlNode::executePlanCallback(
    const std::shared_ptr<ur_manipulation::srv::ExecutePlan::Request> request,
    std::shared_ptr<ur_manipulation::srv::ExecutePlan::Response> response)
{
  RCLCPP_INFO(get_logger(), "Executing planned motion");
  auto execute_result = m_move_group->execute(m_current_plan);
  response->success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
  response->message = response->success ? "Execution successful" : "Execution failed";
}

void MotionControlNode::updateDepthCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(get_logger(), "Update Depth Map request received");
  bool result = updateDepthMap(10000);
  if(result) {
    RCLCPP_INFO(get_logger(), "Depth map updated successfully");
    response->message = "Depth map updated";
  } else {
    RCLCPP_ERROR(get_logger(), "Depth map update failed");
    response->message = "Depth map update failed";
  }
  response->success = result;
  
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
  RCLCPP_ERROR(get_logger(), "[TRACE] updateDepthMap() START");
  using namespace std::chrono;

  auto start = steady_clock::now();
  auto deadline = start + milliseconds(timeout_ms);

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
  auto request = std::make_shared<ur_manipulation::srv::GetPointCloud::Request>();
  auto future = m_depth_client->async_send_request(request);

  // ---- Wait for the response (executor thread will handle it) ----
  auto remaining = deadline - steady_clock::now();
  if (remaining <= milliseconds(0)) {
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

  RCLCPP_ERROR(get_logger(), "[TRACE] Creating planning scene message");
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world.octomap.octomap = octomap_msg;
  RCLCPP_ERROR(get_logger(), "[TRACE] Applying planning scene");
  if(!m_planning_scene_interface->applyPlanningScene(planning_scene_msg)) {
      throw std::runtime_error("Failed to apply planning scene");
  }
  RCLCPP_ERROR(get_logger(), "[TRACE] updateDepthMap() END");
  return true;
}

void MotionControlNode::stopMotionCallback(
    const std::shared_ptr<ur_manipulation::srv::StopMotion::Request>,
    std::shared_ptr<ur_manipulation::srv::StopMotion::Response> response)
{
  RCLCPP_INFO(get_logger(), "Stopping motion");
  m_move_group->stop();
  response->success = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // options.automatically_declare_parameters_from_overrides(true);
  // options.use_intra_process_comms(true);
  options.allow_undeclared_parameters(true);
  auto node = std::make_shared<MotionControlNode>(options);
  node->init();
  node->initMoveGroup();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
