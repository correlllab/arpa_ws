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

  // TF2 Buffer and Listener for pose transformations
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

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
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting planning pipeline ID");
  m_move_group->setPlanningPipelineId("move_group");
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting planner ID");
  m_move_group->setPlannerId("RRTConnectkConfigDefault");
  
  // Log configured planning pipeline and planner
  RCLCPP_INFO(get_logger(), "Planning Pipeline ID: %s", m_move_group->getPlanningPipelineId().c_str());
  RCLCPP_INFO(get_logger(), "Planner ID: %s", m_move_group->getPlannerId().c_str());

  // Set goal position tolerance (meters) - Use default MoveIt tolerances
  m_move_group->setGoalPositionTolerance(0.01);  // 1cm (MoveIt default)

  // Set goal orientation tolerance (radians)
  m_move_group->setGoalOrientationTolerance(0.1);  // ~5.7 degrees (MoveIt default)

  // Set goal joint tolerance (radians)
  m_move_group->setGoalJointTolerance(0.01);  // Default tolerance

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

  if (current_state) {
    RCLCPP_INFO(this->get_logger(), "Robot state received! Motion Control Node fully ready.");
    m_robot_state_ready = true;
    m_init_timer->cancel();  // Stop the timer
    m_init_timer.reset();
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
  m_move_group->setPlanningTime(15.0);  // Increased from 10 to 15 seconds
  m_move_group->setNumPlanningAttempts(10);  // Increased from 5 to 10 attempts

  // Check if we can get current state
  auto current_robot_state = m_move_group->getCurrentState(2.0);
  if (!current_robot_state) {
    RCLCPP_ERROR(get_logger(), "CRITICAL: Cannot get current robot state! Joint states may not be published.");
    response->success = false;
    response->message = "Cannot get robot state - check if robot/simulation is running";
    return;
  }
  RCLCPP_INFO(get_logger(), "Current robot state retrieved successfully");

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
    
  // Log planning configuration
  RCLCPP_INFO(get_logger(), "Planning configuration:");
  RCLCPP_INFO(get_logger(), "  Planning Pipeline ID: %s", m_move_group->getPlanningPipelineId().c_str());
  RCLCPP_INFO(get_logger(), "  Planner ID: %s", m_move_group->getPlannerId().c_str());
  RCLCPP_INFO(get_logger(), "  Planning Time: %.2f s", m_move_group->getPlanningTime());
  RCLCPP_INFO(get_logger(), "  Goal Position Tolerance: %.4f m", m_move_group->getGoalPositionTolerance());
  RCLCPP_INFO(get_logger(), "  Goal Orientation Tolerance: %.4f rad", m_move_group->getGoalOrientationTolerance());
  
  // Log target pose
  RCLCPP_INFO(get_logger(), "Target pose: x=%.4f, y=%.4f, z=%.4f", 
              request->target_pose.pose.position.x,
              request->target_pose.pose.position.y,
              request->target_pose.pose.position.z);
  
  // Use setPoseTarget like RViz does - let MoveIt handle IK internally
  // This is simpler and matches how RViz interactive markers work
  RCLCPP_ERROR(get_logger(), "[TRACE] Setting pose target (MoveIt will solve IK internally)");
  
  // Get planning frame and verify it matches the request frame
  std::string planning_frame = m_move_group->getPlanningFrame();
  RCLCPP_INFO(get_logger(), "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(get_logger(), "Request frame: %s", request->target_pose.header.frame_id.c_str());
  
  // Transform pose to planning frame if needed
  geometry_msgs::msg::PoseStamped target_pose_in_planning_frame;
  target_pose_in_planning_frame.header = request->target_pose.header;
  target_pose_in_planning_frame.pose = request->target_pose.pose;
  
  if (request->target_pose.header.frame_id != planning_frame && 
      !request->target_pose.header.frame_id.empty()) {
    try {
      RCLCPP_INFO(get_logger(), "Transforming pose from %s to %s",
                  request->target_pose.header.frame_id.c_str(), planning_frame.c_str());
      target_pose_in_planning_frame = m_tf_buffer->transform(
          request->target_pose, planning_frame, tf2::durationFromSec(1.0));
      RCLCPP_INFO(get_logger(), "Transformed pose: x=%.4f, y=%.4f, z=%.4f",
                  target_pose_in_planning_frame.pose.position.x,
                  target_pose_in_planning_frame.pose.position.y,
                  target_pose_in_planning_frame.pose.position.z);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to transform pose: %s", ex.what());
      response->success = false;
      response->message = std::string("Frame transform failed: ") + ex.what();
      return;
    }
  }
  
  bool success = false;
  
  if (request->use_cartesian) {
    // Cartesian path planning - straight line motion like interactive marker
    RCLCPP_INFO(get_logger(), "Using Cartesian path planning (straight-line motion)");
    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose_in_planning_frame.pose);
    
    // Compute Cartesian path with 1cm resolution, allow 0% jump threshold
    const double eef_step = 0.01;  // 1cm interpolation step
    const double jump_threshold = 0.0;  // Disable jump threshold
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = m_move_group->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);
    
    RCLCPP_INFO(get_logger(), "Cartesian path computed: %.2f%% achieved", fraction * 100.0);
    
    if (fraction >= 0.95) {  // Accept if 95%+ of path achieved
      m_current_plan.trajectory_ = trajectory;
      success = true;
      response->success = true;
      response->message = "Cartesian planning successful (" + 
                          std::to_string(int(fraction * 100)) + "% achieved)";
      RCLCPP_INFO(get_logger(), "✓ Cartesian planning succeeded! Trajectory has %zu waypoints",
                  m_current_plan.trajectory_.joint_trajectory.points.size());
    } else {
      success = false;
      response->success = false;
      response->message = "Cartesian path only " + std::to_string(int(fraction * 100)) + 
                          "% achievable - obstacle or singularity in path";
      RCLCPP_ERROR(get_logger(), "✗ Cartesian planning failed: only %.2f%% of path achievable", 
                   fraction * 100.0);
    }
  } else {
    // Standard sampling-based planning (RRTConnect)
    RCLCPP_INFO(get_logger(), "Using sampling-based planning (RRTConnect)");
    
    // Set pose target - MoveIt will solve IK internally (like RViz does)
    m_move_group->setPoseTarget(target_pose_in_planning_frame.pose, "tool0");

    RCLCPP_ERROR(get_logger(), "[TRACE] Calling plan()");

    // Get detailed error code
    auto error_code = m_move_group->plan(m_current_plan);
    success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_ERROR(get_logger(), "[TRACE] plan() returned with error code: %d (SUCCESS=%d, PLANNING_FAILED=%d, NO_IK_SOLUTION=%d)", 
                 error_code.val,
                 static_cast<int>(moveit::core::MoveItErrorCode::SUCCESS),
                 static_cast<int>(moveit::core::MoveItErrorCode::PLANNING_FAILED),
                 static_cast<int>(moveit::core::MoveItErrorCode::NO_IK_SOLUTION));

    if (success)
    {
      response->success = true;
      response->message = "Planning successful";
      RCLCPP_INFO(get_logger(), "✓ Planning succeeded! Trajectory has %zu waypoints",
                  m_current_plan.trajectory_.joint_trajectory.points.size());
    }
    else
    {
      response->success = false;
      std::string error_msg;

      // Detailed error messages based on MoveIt error codes
      switch (error_code.val) {
        case moveit::core::MoveItErrorCode::PLANNING_FAILED:
          error_msg = "Planning failed - no solution found. Target may be unreachable.";
          break;
        case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
          error_msg = "Invalid motion plan - check target pose validity.";
          break;
        case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
          error_msg = "Plan invalidated by environment change.";
          break;
        case moveit::core::MoveItErrorCode::CONTROL_FAILED:
          error_msg = "Control failed.";
          break;
        case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
          error_msg = "Unable to acquire sensor data.";
          break;
        case moveit::core::MoveItErrorCode::TIMED_OUT:
          error_msg = "Planning timed out - try increasing planning time.";
          break;
        case moveit::core::MoveItErrorCode::PREEMPTED:
          error_msg = "Planning preempted.";
          break;
        case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
        error_msg = "Start state is in collision! Robot may be colliding with itself or environment.";
        break;
      case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
        error_msg = "Goal state is in collision! Target pose would cause collision.";
        break;
      case moveit::core::MoveItErrorCode::START_STATE_INVALID:
        error_msg = "Start state is invalid - check robot state.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
        error_msg = "Invalid planning group name.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
        error_msg = "Invalid goal constraints.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
        error_msg = "Invalid robot state.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
        error_msg = "Invalid link name.";
        break;
      case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
        error_msg = "No IK solution found - target pose cannot be reached by robot.";
        break;
      default:
        error_msg = "Planning failed with error code: " + std::to_string(error_code.val);
      }

      response->message = error_msg;
      RCLCPP_ERROR(get_logger(), "✗ %s", error_msg.c_str());
    }
  }  // end else (sampling-based planning)
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

  // Get detailed error code
  auto error_code = m_move_group->plan(m_current_plan);
  bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

  RCLCPP_ERROR(get_logger(), "[TRACE] plan() returned with error code: %d", error_code.val);

  if (success)
  {
    response->success = true;
    response->message = "Planning successful";
    RCLCPP_INFO(get_logger(), "✓ Planning succeeded! Trajectory has %zu waypoints",
                m_current_plan.trajectory_.joint_trajectory.points.size());
  }
  else
  {
    response->success = false;
    std::string error_msg;

    // Detailed error messages based on MoveIt error codes
    switch (error_code.val) {
      case moveit::core::MoveItErrorCode::PLANNING_FAILED:
        error_msg = "Planning failed - no solution found. Target may be unreachable.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN:
        error_msg = "Invalid motion plan - check target pose validity.";
        break;
      case moveit::core::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        error_msg = "Plan invalidated by environment change.";
        break;
      case moveit::core::MoveItErrorCode::CONTROL_FAILED:
        error_msg = "Control failed.";
        break;
      case moveit::core::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
        error_msg = "Unable to acquire sensor data.";
        break;
      case moveit::core::MoveItErrorCode::TIMED_OUT:
        error_msg = "Planning timed out - try increasing planning time.";
        break;
      case moveit::core::MoveItErrorCode::PREEMPTED:
        error_msg = "Planning preempted.";
        break;
      case moveit::core::MoveItErrorCode::START_STATE_IN_COLLISION:
        error_msg = "Start state is in collision! Robot may be colliding with itself or environment.";
        break;
      case moveit::core::MoveItErrorCode::GOAL_IN_COLLISION:
        error_msg = "Goal state is in collision! Target pose would cause collision.";
        break;
      case moveit::core::MoveItErrorCode::START_STATE_INVALID:
        error_msg = "Start state is invalid - check robot state.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_GROUP_NAME:
        error_msg = "Invalid planning group name.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
        error_msg = "Invalid goal constraints.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_ROBOT_STATE:
        error_msg = "Invalid robot state.";
        break;
      case moveit::core::MoveItErrorCode::INVALID_LINK_NAME:
        error_msg = "Invalid link name.";
        break;
      case moveit::core::MoveItErrorCode::NO_IK_SOLUTION:
        error_msg = "No IK solution found - target pose cannot be reached by robot.";
        break;
      default:
        error_msg = "Planning failed with error code: " + std::to_string(error_code.val);
    }

    response->message = error_msg;
    RCLCPP_ERROR(get_logger(), "✗ %s", error_msg.c_str());
  }
}

void MotionControlNode::executePlanCallback(
    const std::shared_ptr<ur_manipulation::srv::ExecutePlan::Request> request,
    std::shared_ptr<ur_manipulation::srv::ExecutePlan::Response> response)
{
  RCLCPP_INFO(get_logger(), "Executing planned motion");
  RCLCPP_INFO(get_logger(), "Plan has %zu waypoints", m_current_plan.trajectory_.joint_trajectory.points.size());
  
  if (m_current_plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Cannot execute: plan is empty!");
    response->success = false;
    response->message = "Plan is empty. Please plan first.";
    return;
  }
  
  RCLCPP_INFO(get_logger(), "Calling move_group->execute()...");
  auto execute_result = m_move_group->execute(m_current_plan);
  RCLCPP_INFO(get_logger(), "Execute result: %d (SUCCESS=%d)",
              execute_result.val,
              static_cast<int>(moveit::core::MoveItErrorCode::SUCCESS));
  
  response->success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
  response->message = response->success ? "Execution successful" : "Execution failed";
  
  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "Execution failed with error code: %d", execute_result.val);
  }
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

geometry_msgs::msg::Pose MotionControlNode::planRelativeMotion(
    double dx, double dy, double dz,
    double droll, double dpitch, double dyaw)
{
  // Get current end-effector pose
  auto current_state = m_move_group->getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(get_logger(), "Cannot get current state for relative motion");
    return geometry_msgs::msg::Pose();
  }

  const Eigen::Isometry3d& current_transform = current_state->getGlobalLinkTransform("tool0");
  Eigen::Vector3d current_pos = current_transform.translation();
  Eigen::Quaterniond current_quat(current_transform.rotation());

  // Apply delta to position
  Eigen::Vector3d target_pos = current_pos + Eigen::Vector3d(dx, dy, dz);

  // Apply rotation delta using Euler angles (roll, pitch, yaw) as quaternions
  Eigen::Quaterniond dq_roll(Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX()));
  Eigen::Quaterniond dq_pitch(Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond dq_yaw(Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()));
  
  // Combine rotations: yaw * pitch * roll order
  Eigen::Quaterniond target_quat = current_quat * (dq_roll * dq_pitch * dq_yaw);
  target_quat.normalize();

  // Create target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = target_pos.x();
  target_pose.position.y = target_pos.y();
  target_pose.position.z = target_pos.z();
  target_pose.orientation.x = target_quat.x();
  target_pose.orientation.y = target_quat.y();
  target_pose.orientation.z = target_quat.z();
  target_pose.orientation.w = target_quat.w();

  RCLCPP_INFO(get_logger(), "Relative motion: current (%.3f, %.3f, %.3f) -> target (%.3f, %.3f, %.3f)",
              current_pos.x(), current_pos.y(), current_pos.z(),
              target_pos.x(), target_pos.y(), target_pos.z());

  return target_pose;
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
