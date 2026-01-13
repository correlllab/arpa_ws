#include "arpa_control/motion_control_node.hpp"
#include <chrono>
#include <future>
#include <cmath>

MotionControlNode::MotionControlNode(rclcpp::NodeOptions options)
    : Node("motion_control_node", options)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Constructor Init");

  m_plan_to_pose_service = this->create_service<arpa_control::srv::PlanToPose>(
      "plan_to_pose",
      std::bind(&MotionControlNode::planToPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  m_plan_to_joint_service = this->create_service<arpa_control::srv::PlanToJoint>(
      "plan_to_joint",
      std::bind(&MotionControlNode::planToJointCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_execute_plan_service = this->create_service<arpa_control::srv::ExecutePlan>(
      "execute_plan",
      std::bind(&MotionControlNode::executePlanCallback, this, std::placeholders::_1, std::placeholders::_2));

  m_stop_motion_service = this->create_service<arpa_control::srv::StopMotion>(
      "stop_motion",
      std::bind(&MotionControlNode::stopMotionCallback, this, std::placeholders::_1, std::placeholders::_2));

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

  m_use_depth = false;
  this->declare_parameter("octomap_resolution", 0.03);
  this->declare_parameter("arm_padding", 0.015);
  m_arm_padding = this->get_parameter("arm_padding").as_double();
  m_arm_padding_links = {"forearm_link", "shoulder_link", "upper_arm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "tool_holder_link", "tool_center_link", "tool_head_link"};
  for(auto link : m_arm_padding_links) {
    m_arm_padding_map[link] = m_arm_padding;
  }

  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Constructor Initialized");
}

void MotionControlNode::init()
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Init()");
  m_planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  m_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), "ur_manipulator");
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Init() END");
}

void MotionControlNode::initMoveGroup()
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() START");

  m_move_group->startStateMonitor(1.0);
  m_move_group->setPlannerId("RRTConnectkConfigDefault");
  m_move_group->setPlanningPipelineId("move_group");
  m_move_group->setPlanningTime(10.0);
  m_move_group->setNumPlanningAttempts(10);
  m_move_group->setGoalOrientationTolerance(0.01); 
  m_move_group->setGoalPositionTolerance(0.01);
  moveit_msgs::msg::WorkspaceParameters workspace;
  workspace.header.frame_id = "tool_head_link";
  workspace.min_corner.x = -1.0; workspace.min_corner.y = -1.0; workspace.min_corner.z = -1.0;
  workspace.max_corner.x = 1.0;  workspace.max_corner.y = 1.0;  workspace.max_corner.z = 1.0;
  m_move_group->setWorkspace(
    workspace.min_corner.x, workspace.min_corner.y, workspace.min_corner.z,
    workspace.max_corner.x, workspace.max_corner.y, workspace.max_corner.z
  );

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() END");
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

  // Transform Pose to the World Frame
  geometry_msgs::msg::PoseStamped output_pose;
  try {
      // This will look up the transform and apply it to the pose
      output_pose = m_tf_buffer->transform(request->target_pose, "world", tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
  }

  RCLCPP_INFO(get_logger(), "Planning to target pose: x: %.2f, y: %.2f, z: %.2f",
              output_pose.pose.position.x,
              output_pose.pose.position.y,
              output_pose.pose.position.z);

  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.header.stamp = now();
  static_transform.header.frame_id = "world";
  static_transform.child_frame_id = "target_pose";
  static_transform.transform.translation.x = output_pose.pose.position.x;
  static_transform.transform.translation.y = output_pose.pose.position.y;
  static_transform.transform.translation.z = output_pose.pose.position.z;
  static_transform.transform.rotation = output_pose.pose.orientation;
  m_static_transform_broadcaster->sendTransform(static_transform);

  m_move_group->setStartStateToCurrentState();
  // moveit_msgs::msg::Constraints constraints;
  // moveit_msgs::msg::OrientationConstraint o_constraint;
  // geometry_msgs::msg::Pose current_pose = m_move_group->getCurrentPose("tool_head_link").pose;
  // o_constraint.header.frame_id = "world";
  // o_constraint.link_name = "tool_head_link";
  // o_constraint.orientation = current_pose.orientation; // Keep current orientation
  // o_constraint.absolute_x_axis_tolerance = 0.4; // Allow some wiggle room
  // o_constraint.absolute_y_axis_tolerance = 0.4;
  // o_constraint.absolute_z_axis_tolerance = 3.14; // Allow rotation around the tool axis
  // o_constraint.weight = 1.0;

  // constraints.orientation_constraints.push_back(o_constraint);
  // m_move_group->setPathConstraints(constraints);
  m_move_group->setPoseTarget(output_pose.pose, "tool_head_link");

  bool success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (success)
  {
    response->success = success;
    response->message = response->success ? "Planning successful" : "Planning failed";
  }
  else
  {
    response->success = false;
    response->message = "Planning failed";
  }
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToPoseCallback() END");
}


void MotionControlNode::planToJointCallback(
    const std::shared_ptr<arpa_control::srv::PlanToJoint::Request> request,
    std::shared_ptr<arpa_control::srv::PlanToJoint::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToJointCallback() START");
  m_move_group->setStartStateToCurrentState();

  if (m_use_depth)
  {
    bool reset_depth = resetDepthMap(2000);
    if(!reset_depth) {
      RCLCPP_ERROR(get_logger(), "Depth map failed to update. Abandoning move to joint");
      response->success = false;
      response->message = "Depth reset failed.";
    }
    RCLCPP_INFO(get_logger(), "Updating depth map before planning");
    bool depth_update_success = updateDepthMap(2000);
    if (!depth_update_success)
    {
      RCLCPP_ERROR(get_logger(), "Depth map failed to update. Abandoning move to joint");
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

  std::vector<double> joint_values;
  joint_values.push_back(request->joint1);
  joint_values.push_back(request->joint2);
  joint_values.push_back(request->joint3);
  joint_values.push_back(request->joint4);
  joint_values.push_back(request->joint5);
  joint_values.push_back(request->joint6);
  
  // // Now plan in joint space
  m_move_group->setJointValueTarget(joint_values);
    
  bool success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);

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
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planToJointCallback() END");
}

void MotionControlNode::executePlanCallback(
    const std::shared_ptr<arpa_control::srv::ExecutePlan::Request> request,
    std::shared_ptr<arpa_control::srv::ExecutePlan::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control executePlanCallback() START");
  auto execute_result = m_move_group->execute(m_current_plan);
  response->success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
  response->message = response->success ? "Execution successful" : "Execution failed";
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
