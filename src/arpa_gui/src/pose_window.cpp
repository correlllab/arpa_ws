#include "arpa_gui/pose_window.hpp"

PoseWindow::PoseWindow(rclcpp::Node::SharedPtr node)
    : m_node(node),
      m_tf_buffer(m_node->get_clock()), // TF buffer uses node clock
      m_tf_listener(std::make_shared<tf2_ros::TransformListener>(m_tf_buffer))
{
    auto *layout = new QVBoxLayout;

    auto *form = new QFormLayout;

    m_prismatic_slider = new QSlider(Qt::Horizontal);
    m_prismatic_slider->setMinimum(0.);        // in mm or meters * 1000
    m_prismatic_slider->setMaximum(1845);      // <-- your max stroke
    m_prismatic_slider->setValue(0);
    m_prismatic_slider->setTickInterval(100);
    m_prismatic_slider->setTickPosition(QSlider::TicksBelow);

    m_prismatic_value = new QLineEdit;
    m_prismatic_value->setReadOnly(true);
    m_prismatic_value->setText("0.0");

    QWidget *prismatic_row = new QWidget;
    auto *h = new QHBoxLayout();
    h->addWidget(m_prismatic_slider);
    h->addWidget(m_prismatic_value);
    h->setContentsMargins(0, 0, 0, 0);
    prismatic_row->setLayout(h);
    form->addRow("Prismatic Extension (m):", prismatic_row);

    m_source_frame_selector = new QComboBox;
    m_target_frame_selector = new QComboBox;

    m_source_frame_selector->addItem("base_link");
    m_source_frame_selector->addItem("tool0");

    m_target_frame_selector->addItem("base_link");
    m_target_frame_selector->addItem("tool0");

    // This puts the text *next to* the dropdowns
    form->addRow("Source Frame:", m_source_frame_selector);
    form->addRow("Target Frame:", m_target_frame_selector);

    // Add this layout to the main layout
    layout->addLayout(form);

    // Pose fields
    m_x = new QLineEdit;
    m_y = new QLineEdit;
    m_z = new QLineEdit;
    m_roll = new QLineEdit;
    m_pitch = new QLineEdit;
    m_yaw = new QLineEdit;

    form->addRow("X", m_x);
    form->addRow("Y", m_y);
    form->addRow("Z", m_z);
    form->addRow("Roll", m_roll);
    form->addRow("Pitch", m_pitch);
    form->addRow("Yaw", m_yaw);
    layout->addLayout(form);

    // Buttons
    auto *update_depth = new QPushButton("Update Depth");
    auto *plan = new QPushButton("Plan to Pose");
    auto *exec = new QPushButton("Execute Plan");
    auto *stop = new QPushButton("STOP");
    auto *home = new QPushButton("Home");
    
    layout->addWidget(update_depth);
    layout->addWidget(plan);
    layout->addWidget(exec);
    layout->addWidget(stop);
    layout->addWidget(home);

    setLayout(layout);

    // ROS2 clients
    m_plan_client = m_node->create_client<ur_manipulation::srv::PlanToPose>("plan_to_pose");
    m_update_depth_client = m_node->create_client<std_srvs::srv::Trigger>("update_depth");
    m_exec_client = m_node->create_client<ur_manipulation::srv::ExecutePlan>("execute_plan");
    m_stop_client = m_node->create_client<ur_manipulation::srv::StopMotion>("stop_motion");
    m_linear_actuator_pub = m_node->create_publisher<std_msgs::msg::Float64>("/linear_actuator_joint_position", 10);

    // Button handlers
    connect(update_depth, &QPushButton::clicked, this, &PoseWindow::updateDepth);
    connect(plan, &QPushButton::clicked, this, &PoseWindow::planPose);
    connect(exec, &QPushButton::clicked, this, &PoseWindow::executePlan);
    connect(stop, &QPushButton::clicked, this, &PoseWindow::stopMotion);
    connect(home, &QPushButton::clicked, this, &PoseWindow::goHome);

    // FRAME CHANGE SIGNALS
    connect(m_source_frame_selector, &QComboBox::currentTextChanged,
            this, &PoseWindow::onSourceFrameChanged);

    connect(m_target_frame_selector, &QComboBox::currentTextChanged,
            this, &PoseWindow::onTargetFrameChanged);
    
    connect(m_prismatic_slider, &QSlider::valueChanged,
        this, &PoseWindow::onPrismaticChanged);

    // Set a default home pose 0.020, -0.177, 0.638
    m_home_pose.position.x = 0.020;
    m_home_pose.position.y = -0.177;
    m_home_pose.position.z = 0.638;
    // Rotation: in RPY (radian) [-0.025, 0.002, -0.000]
    tf2::Quaternion home_quat;
    home_quat.setRPY(0.0, 0.0, 0.0); // Facing down
    m_home_pose.orientation.x = home_quat.x();
    m_home_pose.orientation.y = home_quat.y();
    m_home_pose.orientation.z = home_quat.z();
    m_home_pose.orientation.w = home_quat.w();
}

void PoseWindow::planPose()
{
    auto req = std::make_shared<ur_manipulation::srv::PlanToPose::Request>();
    req->target_pose.pose.position.x = m_x->text().toDouble();
    req->target_pose.pose.position.y = m_y->text().toDouble();
    req->target_pose.pose.position.z = m_z->text().toDouble();
    double roll = m_roll->text().toDouble();
    double pitch = m_pitch->text().toDouble();
    double yaw = m_yaw->text().toDouble();

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    req->target_pose.pose.orientation.x = q.x();
    req->target_pose.pose.orientation.y = q.y();
    req->target_pose.pose.orientation.z = q.z();
    req->target_pose.pose.orientation.w = q.w();

    m_plan_client->async_send_request(req);
}

void PoseWindow::executePlan()
{
    auto req = std::make_shared<ur_manipulation::srv::ExecutePlan::Request>();
    m_exec_client->async_send_request(req);
}

void PoseWindow::updateDepth()
{
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    m_update_depth_client->async_send_request(req);
}

void PoseWindow::stopMotion()
{
    auto req = std::make_shared<ur_manipulation::srv::StopMotion::Request>();
    m_stop_client->async_send_request(req);
}

void PoseWindow::onFrameChanged()
{
    if (m_source_frame.empty() || m_target_frame.empty())
        return;

    geometry_msgs::msg::TransformStamped tf_msg;

    try
    {
        tf_msg = m_tf_buffer.lookupTransform(
            m_target_frame,
            m_source_frame,
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(m_node->get_logger(),
                    "TF lookup failed (%s -> %s): %s",
                    m_source_frame.c_str(),
                    m_target_frame.c_str(),
                    ex.what());
        return;
    }

    // Extract pose
    const auto &t = tf_msg.transform.translation;
    const auto &q = tf_msg.transform.rotation;

    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    m_x->setText(QString::number(t.x));
    m_y->setText(QString::number(t.y));
    m_z->setText(QString::number(t.z));
    m_roll->setText(QString::number(roll));
    m_pitch->setText(QString::number(pitch));
    m_yaw->setText(QString::number(yaw));
}

void PoseWindow::onSourceFrameChanged(const QString &frame_qt)
{
    m_source_frame = frame_qt.toStdString();
    onFrameChanged(); // call common function
}

void PoseWindow::onTargetFrameChanged(const QString &frame_qt)
{
    m_target_frame = frame_qt.toStdString();
    onFrameChanged(); // call common function
}

void PoseWindow::goHome()
{
    // ---------- FIXED HOME POSE ----------
    geometry_msgs::msg::Pose home = m_home_pose;

    // ---------- PLAN REQUEST ----------
    auto request = std::make_shared<ur_manipulation::srv::PlanToPose::Request>();
    request->target_pose.pose = home;

    if (!m_plan_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(m_node->get_logger(), "PlanToPose service not available.");
        return;
    }

    auto future = m_plan_client->async_send_request(request);

    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
        RCLCPP_ERROR(m_node->get_logger(), "PlanToPose request timed out.");
        return;
    }

    auto result = future.get();

    if (!result->success) {
        RCLCPP_ERROR(m_node->get_logger(), "Failed to plan home pose.");
        return;
    }

    // ---------- EXECUTE REQUEST ----------
    auto exec_req = std::make_shared<ur_manipulation::srv::ExecutePlan::Request>();

    if (!m_exec_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(m_node->get_logger(), "ExecutePlan service not available.");
        return;
    }

    auto exec_future = m_exec_client->async_send_request(exec_req);

    if (exec_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
        RCLCPP_ERROR(m_node->get_logger(), "ExecutePlan request timed out.");
        return;
    }

    RCLCPP_INFO(m_node->get_logger(), "Robot moving to HOME pose.");
}

void PoseWindow::onPrismaticChanged(int value)
{
    // Update numeric box next to slider
    m_prismatic_value->setText(QString::number(value));
    double position_m = static_cast<double>(value) / 1000.0;

    RCLCPP_INFO(m_node->get_logger(),
                "Prismatic joint command: %.3f m", position_m);

    std_msgs::msg::Float64 cmd;
    cmd.data = position_m;  // 15 cm extension
    m_linear_actuator_pub->publish(cmd);
}



