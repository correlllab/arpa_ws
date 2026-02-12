#include "arpa_control/motion_control_node.hpp"
#include <chrono>
#include <future>
#include <cmath>
#include <thread>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <Eigen/Geometry>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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

  m_plan_linear_actuator_service = this->create_service<arpa_control::srv::PlanLinearActuator>(
      "plan_linear_actuator",
      std::bind(&MotionControlNode::planLinearActuatorCallback, this, std::placeholders::_1, std::placeholders::_2));

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
  this->declare_parameter("use_corridor_constraint", false);  // if true, constrain RRT to corridor; off by default (causes plan rejections)
  this->declare_parameter("corridor_padding", 0.02);   // extra length (m) at each end of corridor
  this->declare_parameter("corridor_cross_section", 0.12);  // half-width (m) perpendicular to segment
  m_arm_padding = this->get_parameter("arm_padding").as_double();
  m_arm_padding_links = {"forearm_link", "shoulder_link", "upper_arm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0", "tool_holder_link", "runner_link", "tool_head_link"};
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
    shared_from_this(), "ur16e_on_gantry");
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control Init() END");
}

void MotionControlNode::initMoveGroup()
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() START");

  m_move_group->startStateMonitor(3.0);  // Increased timeout to 3 seconds for better reliability
  m_move_group->setPlannerId("RRTConnectkConfigDefault");
  m_move_group->setPlanningPipelineId("move_group");
  m_move_group->setPlanningTime(5.0);
  m_move_group->setNumPlanningAttempts(10);
  m_move_group->setMaxVelocityScalingFactor(0.1);
  m_move_group->setMaxAccelerationScalingFactor(0.1);
  m_move_group->setGoalPositionTolerance(0.01);
  m_move_group->setGoalOrientationTolerance(0.01); 

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control initMoveGroup() END");
}

bool MotionControlNode::configureForPlanning(geometry_msgs::msg::Pose target_pose)
{
  // Get current robot state as IK seed (biases solution toward current config)
  // Retry up to 3 times if state is not available
  auto robot_state = m_move_group->getCurrentState();
  int retries = 0;
  while (!robot_state && retries < 3) {
    RCLCPP_WARN(get_logger(), "Failed to get current robot state, retrying... (attempt %d/3)", retries + 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    robot_state = m_move_group->getCurrentState();
    retries++;
  }
  
  if (!robot_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state after 3 retries");
    return false;
  }

  // Compute IK to convert pose to joint values (longer timeout for RRT/transfer poses)
  const auto* joint_model_group = robot_state->getJointModelGroup(m_move_group->getName());
  bool ik_success = robot_state->setFromIK(
      joint_model_group,
      target_pose,
      m_move_group->getEndEffectorLink(),
      2.0);  // timeout in seconds (was 0.1; 2.0 helps for 7-DOF transfer targets)

  if (!ik_success) {
    RCLCPP_ERROR(get_logger(), "IK failed for target pose");
    return false;
  }

  // Set joint value target from IK solution
  m_move_group->setJointValueTarget(*robot_state);
  m_move_group->setStartStateToCurrentState();

  return true;
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
    const double eef_step = 0.005;  // 5mm interpolation resolution
    const double jump_threshold = 0.0;  // Disable jump detection
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
  } else {
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
    const bool use_corridor = this->get_parameter("use_corridor_constraint").as_bool();
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

    if (!configureForPlanning(target_pose_in_planning_frame.pose)) {
      m_move_group->clearPathConstraints();
      response->success = false;
      response->message = "Failed to configure for planning";
      return;
    }

    bool success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
      // Fallback: retry once without corridor constraint (path may have been invalid due to tight corridor)
      m_move_group->clearPathConstraints();
      RCLCPP_INFO(get_logger(), "RRT with corridor failed, retrying without path constraints");
      if (configureForPlanning(target_pose_in_planning_frame.pose)) {
        success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      }
    }
    m_move_group->clearPathConstraints();
    response->success = success;
    response->message = success ? "Planning successful" : "Planning failed";
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

  // ur16e_on_gantry has 7 joints; PlanToJoint provides 6 arm joints. Keep linear actuator at current position.
  auto robot_state = m_move_group->getCurrentState();
  if (!robot_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state");
    response->success = false;
    response->message = "Failed to get current robot state";
    return;
  }

  std::map<std::string, double> joint_targets;
  const double* gantry_pos = robot_state->getJointPositions("linear_actuator_to_linear_actuator_plate_joint");
  if (gantry_pos) {
    joint_targets["linear_actuator_to_linear_actuator_plate_joint"] = gantry_pos[0];
  }
  joint_targets["shoulder_pan_joint"] = request->joint1;
  joint_targets["shoulder_lift_joint"] = request->joint2;
  joint_targets["elbow_joint"] = request->joint3;
  joint_targets["wrist_1_joint"] = request->joint4;
  joint_targets["wrist_2_joint"] = request->joint5;
  joint_targets["wrist_3_joint"] = request->joint6;

  m_move_group->setJointValueTarget(joint_targets);
    
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

void MotionControlNode::planLinearActuatorCallback(
    const std::shared_ptr<arpa_control::srv::PlanLinearActuator::Request> request,
    std::shared_ptr<arpa_control::srv::PlanLinearActuator::Response> response)
{
  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planLinearActuatorCallback() START");
  RCLCPP_INFO(get_logger(), "Planning linear actuator to position: %.3f m", request->position);

  m_move_group->setStartStateToCurrentState();

  // Get current robot state
  auto robot_state = m_move_group->getCurrentState();
  if (!robot_state) {
    RCLCPP_ERROR(get_logger(), "Failed to get current robot state");
    response->success = false;
    response->message = "Failed to get current robot state";
    return;
  }

  // Set only the linear actuator joint target, keep all other joints at current position
  std::map<std::string, double> joint_targets;
  joint_targets["linear_actuator_to_linear_actuator_plate_joint"] = request->position;
  joint_targets["shoulder_pan_joint"] = robot_state->getJointPositions("shoulder_pan_joint")[0];
  joint_targets["shoulder_lift_joint"] = robot_state->getJointPositions("shoulder_lift_joint")[0];
  joint_targets["elbow_joint"] = robot_state->getJointPositions("elbow_joint")[0];
  joint_targets["wrist_1_joint"] = robot_state->getJointPositions("wrist_1_joint")[0];
  joint_targets["wrist_2_joint"] = robot_state->getJointPositions("wrist_2_joint")[0];
  joint_targets["wrist_3_joint"] = robot_state->getJointPositions("wrist_3_joint")[0];

  m_move_group->setJointValueTarget(joint_targets);

  bool success = (m_move_group->plan(m_current_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  response->success = success;
  response->message = success ? "Linear actuator planning successful" : "Linear actuator planning failed";

  RCLCPP_INFO(get_logger(), "[TRACE] Motion Control planLinearActuatorCallback() END");
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
