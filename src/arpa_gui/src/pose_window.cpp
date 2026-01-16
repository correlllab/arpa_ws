#include "arpa_gui/pose_window.hpp"
#include <QDateTime>
#include <QScrollBar>

// Joint names for UR robot
static const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

static const std::vector<QString> JOINT_DISPLAY_NAMES = {
    "Shoulder Pan",
    "Shoulder Lift",
    "Elbow",
    "Wrist 1",
    "Wrist 2",
    "Wrist 3"
};

PoseWindow::PoseWindow(rclcpp::Node::SharedPtr node)
    : m_node(node),
      m_tf_buffer(node->get_clock())
{
    // Initialize TF listener after buffer is constructed
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(m_tf_buffer);
    setWindowTitle("ARPA Robot Control Panel");
    setMinimumSize(500, 800);

    m_current_joint_values.resize(6, 0.0);

    setupUI();
    setupConnections();
    applyStylesheet();

    // ROS2 clients
    m_plan_client = m_node->create_client<arpa_control::srv::PlanToPose>("plan_to_pose");
    m_update_depth_client = m_node->create_client<std_srvs::srv::Trigger>("update_depth");
    m_exec_client = m_node->create_client<arpa_control::srv::ExecutePlan>("execute_plan");
    m_stop_client = m_node->create_client<arpa_control::srv::StopMotion>("stop_motion");
    // Linear actuator controller - supports both manual slider control and MoveIt 7-DOF planning
    m_linear_actuator_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>("/linear_actuator_controller/commands", 10);

    // Subscribe to joint states
    m_joint_state_sub = m_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&PoseWindow::jointStateCallback, this, std::placeholders::_1));

    // Set default home pose
    m_home_pose.position.x = 0.020;
    m_home_pose.position.y = -0.177;
    m_home_pose.position.z = 0.638;
    tf2::Quaternion home_quat;
    home_quat.setRPY(0.0, 0.0, 0.0);
    m_home_pose.orientation.x = home_quat.x();
    m_home_pose.orientation.y = home_quat.y();
    m_home_pose.orientation.z = home_quat.z();
    m_home_pose.orientation.w = home_quat.w();

    // Setup timer for periodic pose updates
    m_update_timer = new QTimer(this);
    connect(m_update_timer, &QTimer::timeout, this, &PoseWindow::updateCurrentPose);
    m_update_timer->start(100); // Update at 10Hz

    logStatus("ARPA Control Panel initialized");
}

void PoseWindow::setupUI()
{
    auto *mainLayout = new QVBoxLayout;
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(15, 15, 15, 15);

    // ============ CURRENT POSE GROUP ============
    m_current_pose_group = new QGroupBox("Current End-Effector Pose");
    auto *currentPoseLayout = new QGridLayout;

    // Position labels
    currentPoseLayout->addWidget(new QLabel("X:"), 0, 0);
    m_current_x = new QLabel("0.000");
    m_current_x->setAlignment(Qt::AlignRight);
    currentPoseLayout->addWidget(m_current_x, 0, 1);
    currentPoseLayout->addWidget(new QLabel("m"), 0, 2);

    currentPoseLayout->addWidget(new QLabel("Y:"), 0, 3);
    m_current_y = new QLabel("0.000");
    m_current_y->setAlignment(Qt::AlignRight);
    currentPoseLayout->addWidget(m_current_y, 0, 4);
    currentPoseLayout->addWidget(new QLabel("m"), 0, 5);

    currentPoseLayout->addWidget(new QLabel("Z:"), 0, 6);
    m_current_z = new QLabel("0.000");
    m_current_z->setAlignment(Qt::AlignRight);
    currentPoseLayout->addWidget(m_current_z, 0, 7);
    currentPoseLayout->addWidget(new QLabel("m"), 0, 8);

    // Orientation labels
    currentPoseLayout->addWidget(new QLabel("Roll:"), 1, 0);
    m_current_roll = new QLabel("0.000");
    m_current_roll->setAlignment(Qt::AlignRight);
    currentPoseLayout->addWidget(m_current_roll, 1, 1);
    currentPoseLayout->addWidget(new QLabel("rad"), 1, 2);

    currentPoseLayout->addWidget(new QLabel("Pitch:"), 1, 3);
    m_current_pitch = new QLabel("0.000");
    m_current_pitch->setAlignment(Qt::AlignRight);
    currentPoseLayout->addWidget(m_current_pitch, 1, 4);
    currentPoseLayout->addWidget(new QLabel("rad"), 1, 5);

    currentPoseLayout->addWidget(new QLabel("Yaw:"), 1, 6);
    m_current_yaw = new QLabel("0.000");
    m_current_yaw->setAlignment(Qt::AlignRight);
    currentPoseLayout->addWidget(m_current_yaw, 1, 7);
    currentPoseLayout->addWidget(new QLabel("rad"), 1, 8);

    m_current_pose_group->setLayout(currentPoseLayout);
    mainLayout->addWidget(m_current_pose_group);

    // ============ JOINT STATES GROUP ============
    m_joint_states_group = new QGroupBox("Joint States");
    auto *jointLayout = new QGridLayout;

    for (int i = 0; i < 6; ++i) {
        m_joint_labels[i] = new QLabel(JOINT_DISPLAY_NAMES[i] + ": 0.00 rad");
        m_joint_bars[i] = new QProgressBar;
        m_joint_bars[i]->setRange(-314, 314); // -pi to pi scaled by 100
        m_joint_bars[i]->setValue(0);
        m_joint_bars[i]->setTextVisible(false);
        m_joint_bars[i]->setFixedHeight(15);

        jointLayout->addWidget(m_joint_labels[i], i, 0);
        jointLayout->addWidget(m_joint_bars[i], i, 1);
    }

    m_joint_states_group->setLayout(jointLayout);
    mainLayout->addWidget(m_joint_states_group);

    // ============ PRISMATIC CONTROL GROUP ============
    m_prismatic_group = new QGroupBox("Linear Actuator Control");
    auto *prismaticLayout = new QHBoxLayout;

    m_prismatic_slider = new QSlider(Qt::Horizontal);
    m_prismatic_slider->setMinimum(0);
    m_prismatic_slider->setMaximum(1845);
    m_prismatic_slider->setValue(0);
    m_prismatic_slider->setTickInterval(100);
    m_prismatic_slider->setTickPosition(QSlider::TicksBelow);

    m_prismatic_value = new QLineEdit("0.000");
    m_prismatic_value->setReadOnly(true);
    m_prismatic_value->setFixedWidth(80);
    m_prismatic_value->setAlignment(Qt::AlignRight);

    prismaticLayout->addWidget(new QLabel("Extension:"));
    prismaticLayout->addWidget(m_prismatic_slider, 1);
    prismaticLayout->addWidget(m_prismatic_value);
    prismaticLayout->addWidget(new QLabel("m"));

    m_prismatic_group->setLayout(prismaticLayout);
    mainLayout->addWidget(m_prismatic_group);

    // ============ TARGET POSE GROUP ============
    m_target_pose_group = new QGroupBox("Relative Motion (Deltas from Current Pose)");
    auto *targetLayout = new QFormLayout;

    // Frame selectors
    auto *frameLayout = new QHBoxLayout;
    m_source_frame_selector = new QComboBox;
    m_target_frame_selector = new QComboBox;
    m_source_frame_selector->addItems({"base_link", "tool0", "floor_link"});
    m_target_frame_selector->addItems({"base_link", "tool0", "floor_link"});
    m_target_frame_selector->setCurrentIndex(1); // default to tool0

    frameLayout->addWidget(new QLabel("From:"));
    frameLayout->addWidget(m_source_frame_selector);
    frameLayout->addWidget(new QLabel("To:"));
    frameLayout->addWidget(m_target_frame_selector);
    targetLayout->addRow(frameLayout);

    // Position inputs (RELATIVE DELTAS)
    auto *posLayout = new QHBoxLayout;
    m_x = new QLineEdit("0.0");
    m_y = new QLineEdit("0.0");
    m_z = new QLineEdit("0.0");
    m_x->setFixedWidth(80);
    m_y->setFixedWidth(80);
    m_z->setFixedWidth(80);

    posLayout->addWidget(new QLabel("ΔX:"));
    posLayout->addWidget(m_x);
    posLayout->addWidget(new QLabel("ΔY:"));
    posLayout->addWidget(m_y);
    posLayout->addWidget(new QLabel("ΔZ:"));
    posLayout->addWidget(m_z);
    targetLayout->addRow("Position Delta (m):", posLayout);

    // Orientation inputs (RELATIVE DELTAS)
    auto *orientLayout = new QHBoxLayout;
    m_roll = new QLineEdit("0.0");
    m_pitch = new QLineEdit("0.0");
    m_yaw = new QLineEdit("0.0");
    m_roll->setFixedWidth(80);
    m_pitch->setFixedWidth(80);
    m_yaw->setFixedWidth(80);

    orientLayout->addWidget(new QLabel("ΔR:"));
    orientLayout->addWidget(m_roll);
    orientLayout->addWidget(new QLabel("ΔP:"));
    orientLayout->addWidget(m_pitch);
    orientLayout->addWidget(new QLabel("ΔY:"));
    orientLayout->addWidget(m_yaw);
    targetLayout->addRow("Orientation Delta (rad):", orientLayout);

    m_target_pose_group->setLayout(targetLayout);
    mainLayout->addWidget(m_target_pose_group);

    // ============ CONTROL BUTTONS ============
    auto *buttonGroup = new QGroupBox("Control");
    auto *buttonLayout = new QGridLayout;

    m_plan_btn = new QPushButton("Plan");
    m_exec_btn = new QPushButton("Execute");
    m_stop_btn = new QPushButton("STOP");
    m_home_btn = new QPushButton("Home");
    m_update_depth_btn = new QPushButton("Update Depth");
    m_test_btn = new QPushButton("TEST: Move 1cm Up");
    m_cartesian_checkbox = new QCheckBox("Cartesian (straight-line)");
    m_cartesian_checkbox->setChecked(true);  // Default to Cartesian for smoother motion
    m_cartesian_checkbox->setToolTip("Use straight-line path planning instead of sampling-based (RRTConnect)");

    m_plan_btn->setMinimumHeight(40);
    m_exec_btn->setMinimumHeight(40);
    m_stop_btn->setMinimumHeight(50);
    m_home_btn->setMinimumHeight(40);
    m_update_depth_btn->setMinimumHeight(40);
    m_test_btn->setMinimumHeight(40);

    buttonLayout->addWidget(m_cartesian_checkbox, 0, 0, 1, 2);  // Span 2 columns
    buttonLayout->addWidget(m_plan_btn, 1, 0);
    buttonLayout->addWidget(m_exec_btn, 1, 1);
    buttonLayout->addWidget(m_home_btn, 2, 0);
    buttonLayout->addWidget(m_update_depth_btn, 2, 1);
    buttonLayout->addWidget(m_test_btn, 3, 0, 1, 2);
    buttonLayout->addWidget(m_stop_btn, 4, 0, 1, 2);

    buttonGroup->setLayout(buttonLayout);
    mainLayout->addWidget(buttonGroup);

    // ============ STATUS LOG ============
    m_status_group = new QGroupBox("Status Log");
    auto *statusLayout = new QVBoxLayout;

    m_status_log = new QTextEdit;
    m_status_log->setReadOnly(true);
    m_status_log->setMaximumHeight(120);
    m_status_log->setLineWrapMode(QTextEdit::WidgetWidth);

    statusLayout->addWidget(m_status_log);
    m_status_group->setLayout(statusLayout);
    mainLayout->addWidget(m_status_group);

    mainLayout->addStretch();
    setLayout(mainLayout);
}

void PoseWindow::setupConnections()
{
    connect(m_plan_btn, &QPushButton::clicked, this, &PoseWindow::planPose);
    connect(m_exec_btn, &QPushButton::clicked, this, &PoseWindow::executePlan);
    connect(m_stop_btn, &QPushButton::clicked, this, &PoseWindow::stopMotion);
    connect(m_home_btn, &QPushButton::clicked, this, &PoseWindow::goHome);
    connect(m_update_depth_btn, &QPushButton::clicked, this, &PoseWindow::updateDepth);
    connect(m_test_btn, &QPushButton::clicked, this, &PoseWindow::testMoveUp);

    // Use lambda to avoid calling onFrameChanged during startup when TF isn't ready
    connect(m_source_frame_selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { onSourceFrameChanged(m_source_frame_selector->currentText()); });
    connect(m_target_frame_selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { onTargetFrameChanged(m_target_frame_selector->currentText()); });

    connect(m_prismatic_slider, &QSlider::valueChanged,
            this, &PoseWindow::onPrismaticChanged);
}

void PoseWindow::applyStylesheet()
{
    QString stylesheet = R"(
        QWidget {
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 11px;
            background-color: white;
            color: black;
        }

        QGroupBox {
            font-weight: bold;
            font-size: 12px;
            border: 2px solid black;
            border-radius: 8px;
            margin-top: 12px;
            padding-top: 10px;
            background-color: white;
            color: black;
        }

        QGroupBox::title {
            subcontrol-origin: margin;
            left: 15px;
            padding: 0 8px;
            color: black;
        }

        QLabel {
            color: black;
        }

        QLineEdit {
            border: 1px solid black;
            border-radius: 4px;
            padding: 5px;
            background-color: white;
            color: black;
        }

        QLineEdit:focus {
            border: 2px solid black;
        }

        QLineEdit:read-only {
            background-color: #f0f0f0;
            color: black;
            font-weight: bold;
        }

        QPushButton {
            background-color: black;
            color: white;
            border: 1px solid black;
            border-radius: 6px;
            padding: 8px 16px;
            font-weight: bold;
        }

        QPushButton:hover {
            background-color: #333333;
        }

        QPushButton:pressed {
            background-color: #666666;
        }

        QComboBox {
            border: 1px solid black;
            border-radius: 4px;
            padding: 5px;
            background-color: white;
            color: black;
        }

        QSlider::groove:horizontal {
            border: 1px solid black;
            height: 8px;
            background: #e0e0e0;
            border-radius: 4px;
        }

        QSlider::handle:horizontal {
            background: black;
            border: 1px solid black;
            width: 18px;
            margin: -5px 0;
            border-radius: 9px;
        }

        QSlider::handle:horizontal:hover {
            background: #333333;
        }

        QProgressBar {
            border: 1px solid black;
            border-radius: 4px;
            background-color: #e0e0e0;
        }

        QProgressBar::chunk {
            background-color: black;
            border-radius: 3px;
        }

        /* Status log keeps colors for error/success messages */
        QTextEdit {
            border: 1px solid black;
            border-radius: 4px;
            background-color: #2c3e50;
            color: #ecf0f1;
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 10px;
        }
    )";

    setStyleSheet(stylesheet);
}

void PoseWindow::logStatus(const QString &message, bool isError)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString coloredMsg;

    if (isError) {
        coloredMsg = QString("<span style='color: #e74c3c;'>[%1] ERROR: %2</span>").arg(timestamp, message);
    } else {
        coloredMsg = QString("<span style='color: #2ecc71;'>[%1] %2</span>").arg(timestamp, message);
    }

    m_status_log->append(coloredMsg);

    // Auto-scroll to bottom
    QScrollBar *sb = m_status_log->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void PoseWindow::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Update joint values from message
    for (size_t i = 0; i < JOINT_NAMES.size() && i < 6; ++i) {
        for (size_t j = 0; j < msg->name.size(); ++j) {
            if (msg->name[j] == JOINT_NAMES[i] && j < msg->position.size()) {
                m_current_joint_values[i] = msg->position[j];
                break;
            }
        }
    }

    // Update UI must be done in Qt thread - use QMetaObject::invokeMethod
    QMetaObject::invokeMethod(this, [this]() {
        for (int i = 0; i < 6; ++i) {
            double val = m_current_joint_values[i];
            m_joint_labels[i]->setText(QString("%1: %2 rad")
                .arg(JOINT_DISPLAY_NAMES[i])
                .arg(val, 0, 'f', 3));
            m_joint_bars[i]->setValue(static_cast<int>(val * 100));
        }
    }, Qt::QueuedConnection);
}

void PoseWindow::updateCurrentPose()
{
    try {
        geometry_msgs::msg::TransformStamped tf_msg = m_tf_buffer.lookupTransform(
            "base_link", "tool0", tf2::TimePointZero);

        const auto &t = tf_msg.transform.translation;
        const auto &q = tf_msg.transform.rotation;

        m_current_x->setText(QString::number(t.x, 'f', 4));
        m_current_y->setText(QString::number(t.y, 'f', 4));
        m_current_z->setText(QString::number(t.z, 'f', 4));

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        m_current_roll->setText(QString::number(roll, 'f', 4));
        m_current_pitch->setText(QString::number(pitch, 'f', 4));
        m_current_yaw->setText(QString::number(yaw, 'f', 4));

    } catch (const tf2::TransformException &ex) {
        // Silently ignore - TF may not be available yet
    }
}

void PoseWindow::planPose()
{
    logStatus("Planning to target pose...");

    // Get current end-effector pose
    geometry_msgs::msg::Pose current_pose;
    try {
        geometry_msgs::msg::TransformStamped tf_msg = m_tf_buffer.lookupTransform(
            "base_link", "tool0", tf2::TimePointZero);

        current_pose.position.x = tf_msg.transform.translation.x;
        current_pose.position.y = tf_msg.transform.translation.y;
        current_pose.position.z = tf_msg.transform.translation.z;
        current_pose.orientation = tf_msg.transform.rotation;
    } catch (const tf2::TransformException &ex) {
        logStatus(QString("Failed to get current pose: %1").arg(ex.what()), true);
        return;
    }

    // Parse input values as RELATIVE offsets (deltas)
    double delta_x = m_x->text().toDouble();
    double delta_y = m_y->text().toDouble();
    double delta_z = m_z->text().toDouble();
    double delta_roll = m_roll->text().toDouble();
    double delta_pitch = m_pitch->text().toDouble();
    double delta_yaw = m_yaw->text().toDouble();

    // Compute target pose = current + deltas
    auto req = std::make_shared<arpa_control::srv::PlanToPose::Request>();
    req->target_pose.header.frame_id = m_target_frame_selector->currentText().toStdString();
    req->target_pose.pose.position.x = delta_x;
    req->target_pose.pose.position.y = delta_y;
    req->target_pose.pose.position.z = delta_z;

    // If orientation deltas are zero, preserve current orientation
    if (std::abs(delta_roll) < 1e-6 && std::abs(delta_pitch) < 1e-6 && std::abs(delta_yaw) < 1e-6) {
        // Keep current orientation
        req->target_pose.pose.orientation = current_pose.orientation;
        logStatus("Using current orientation (no orientation deltas specified)");
    } else {
        // Apply orientation deltas
        tf2::Quaternion current_quat(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);

        double current_roll, current_pitch, current_yaw;
        tf2::Matrix3x3(current_quat).getRPY(current_roll, current_pitch, current_yaw);

        tf2::Quaternion target_quat;
        target_quat.setRPY(
            current_roll + delta_roll,
            current_pitch + delta_pitch,
            current_yaw + delta_yaw);

        req->target_pose.pose.orientation.x = target_quat.x();
        req->target_pose.pose.orientation.y = target_quat.y();
        req->target_pose.pose.orientation.z = target_quat.z();
        req->target_pose.pose.orientation.w = target_quat.w();

        logStatus(QString("Applying orientation deltas: R=%1 P=%2 Y=%3").arg(
            delta_roll, 0, 'f', 3).arg(delta_pitch, 0, 'f', 3).arg(delta_yaw, 0, 'f', 3));
    }

    logStatus(QString("Target (absolute): X=%1 Y=%2 Z=%3 (from deltas: dx=%4 dy=%5 dz=%6)").arg(
        req->target_pose.pose.position.x, 0, 'f', 3).arg(
        req->target_pose.pose.position.y, 0, 'f', 3).arg(
        req->target_pose.pose.position.z, 0, 'f', 3).arg(
        delta_x, 0, 'f', 3).arg(delta_y, 0, 'f', 3).arg(delta_z, 0, 'f', 3));

    // Use Cartesian (straight-line) planning if checkbox is checked
    req->use_cartesian = m_cartesian_checkbox->isChecked();
    logStatus(QString("Planning mode: %1").arg(req->use_cartesian ? "Cartesian (straight-line)" : "Sampling-based (RRTConnect)"));

    auto future = m_plan_client->async_send_request(req,
        [this](rclcpp::Client<arpa_control::srv::PlanToPose>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                QMetaObject::invokeMethod(this, [this]() {
                    logStatus("Planning successful!");
                });
            } else {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("Planning failed: " + QString::fromStdString(result->message), true);
                });
            }
        });
}

void PoseWindow::executePlan()
{
    logStatus("Executing planned motion...");

    auto req = std::make_shared<arpa_control::srv::ExecutePlan::Request>();
    auto future = m_exec_client->async_send_request(req,
        [this](rclcpp::Client<arpa_control::srv::ExecutePlan>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                QMetaObject::invokeMethod(this, [this]() {
                    logStatus("Execution complete!");
                });
            } else {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("Execution failed: " + QString::fromStdString(result->message), true);
                });
            }
        });
}

void PoseWindow::updateDepth()
{
    logStatus("Updating depth map...");

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = m_update_depth_client->async_send_request(req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                QMetaObject::invokeMethod(this, [this]() {
                    logStatus("Depth map updated!");
                });
            } else {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("Depth update failed: " + QString::fromStdString(result->message), true);
                });
            }
        });
}

void PoseWindow::stopMotion()
{
    logStatus("STOPPING MOTION!", true);

    auto req = std::make_shared<arpa_control::srv::StopMotion::Request>();
    m_stop_client->async_send_request(req,
        [this](rclcpp::Client<arpa_control::srv::StopMotion>::SharedFuture future) {
            auto result = future.get();
            QMetaObject::invokeMethod(this, [this]() {
                logStatus("Motion stopped");
            });
        });
}

void PoseWindow::testMoveUp()
{
    logStatus("TEST: Planning to move 1cm up in Z direction...");

    // Get current end-effector pose
    geometry_msgs::msg::Pose current_pose;
    try {
        geometry_msgs::msg::TransformStamped tf_msg = m_tf_buffer.lookupTransform(
            "base_link", "tool0", tf2::TimePointZero);

        current_pose.position.x = tf_msg.transform.translation.x;
        current_pose.position.y = tf_msg.transform.translation.y;
        current_pose.position.z = tf_msg.transform.translation.z;
        current_pose.orientation = tf_msg.transform.rotation;
    } catch (const tf2::TransformException &ex) {
        logStatus(QString("Failed to get current pose: %1").arg(ex.what()), true);
        return;
    }

    // Target pose: current + 1cm in Z
    auto req = std::make_shared<arpa_control::srv::PlanToPose::Request>();
    req->target_pose.header.frame_id = "base_link";
    req->target_pose.pose.position.x = current_pose.position.x;
    req->target_pose.pose.position.y = current_pose.position.y;
    req->target_pose.pose.position.z = current_pose.position.z + 0.01;  // 1cm up
    req->target_pose.pose.orientation = current_pose.orientation;  // Keep current orientation
    req->use_cartesian = true;  // Always use Cartesian for small test movements

    logStatus(QString("Current Z: %1 m, Target Z: %2 m (delta: +0.01 m)").arg(
        current_pose.position.z, 0, 'f', 3).arg(req->target_pose.pose.position.z, 0, 'f', 3));

    // Plan and execute automatically (using Cartesian path)
    auto future = m_plan_client->async_send_request(req,
        [this](rclcpp::Client<arpa_control::srv::PlanToPose>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                QMetaObject::invokeMethod(this, [this]() {
                    logStatus("TEST: Planning successful! Executing...");
                    // Automatically execute after planning succeeds
                    executePlan();
                });
            } else {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("TEST: Planning failed: " + QString::fromStdString(result->message), true);
                });
            }
        });
}

void PoseWindow::onFrameChanged()
{
    if (m_source_frame.empty() || m_target_frame.empty())
        return;

    try {
        geometry_msgs::msg::TransformStamped tf_msg = m_tf_buffer.lookupTransform(
            m_target_frame, m_source_frame, tf2::TimePointZero);

        const auto &t = tf_msg.transform.translation;
        const auto &q = tf_msg.transform.rotation;

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        m_x->setText(QString::number(t.x, 'f', 4));
        m_y->setText(QString::number(t.y, 'f', 4));
        m_z->setText(QString::number(t.z, 'f', 4));
        m_roll->setText(QString::number(roll, 'f', 4));
        m_pitch->setText(QString::number(pitch, 'f', 4));
        m_yaw->setText(QString::number(yaw, 'f', 4));

        logStatus(QString("Frame transform loaded: %1 -> %2")
            .arg(QString::fromStdString(m_source_frame))
            .arg(QString::fromStdString(m_target_frame)));

    } catch (const tf2::TransformException &ex) {
        logStatus(QString("TF lookup failed: %1").arg(ex.what()), true);
    }
}

void PoseWindow::onSourceFrameChanged(const QString &frame_qt)
{
    m_source_frame = frame_qt.toStdString();
    onFrameChanged();
}

void PoseWindow::onTargetFrameChanged(const QString &frame_qt)
{
    m_target_frame = frame_qt.toStdString();
    onFrameChanged();
}

void PoseWindow::goHome()
{
    logStatus("Moving to HOME position...");

    auto request = std::make_shared<arpa_control::srv::PlanToPose::Request>();
    request->target_pose.header.frame_id = "base_link";
    request->target_pose.pose = m_home_pose;
    request->use_cartesian = false;  // Use sampling-based for large home movements

    // Use non-blocking async request
    m_plan_client->async_send_request(request,
        [this](rclcpp::Client<arpa_control::srv::PlanToPose>::SharedFuture future) {
            auto result = future.get();
            if (!result->success) {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("Failed to plan home pose: " + QString::fromStdString(result->message), true);
                }, Qt::QueuedConnection);
                return;
            }

            QMetaObject::invokeMethod(this, [this]() {
                logStatus("Home pose planned, executing...");
            }, Qt::QueuedConnection);

            // Execute the plan
            auto exec_req = std::make_shared<arpa_control::srv::ExecutePlan::Request>();
            m_exec_client->async_send_request(exec_req,
                [this](rclcpp::Client<arpa_control::srv::ExecutePlan>::SharedFuture exec_future) {
                    auto exec_result = exec_future.get();
                    if (exec_result->success) {
                        QMetaObject::invokeMethod(this, [this]() {
                            logStatus("Robot moved to HOME pose");
                        }, Qt::QueuedConnection);
                    } else {
                        QMetaObject::invokeMethod(this, [this, exec_result]() {
                            logStatus("Failed to execute home: " + QString::fromStdString(exec_result->message), true);
                        }, Qt::QueuedConnection);
                    }
                });
        });
}

void PoseWindow::onPrismaticChanged(int value)
{
    // Convert slider value to negative position (joint limits are -1.845 to 0)
    double position_m = -static_cast<double>(value) / 1000.0;
    m_prismatic_value->setText(QString::number(-position_m, 'f', 3));

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.push_back(position_m);
    m_linear_actuator_pub->publish(cmd);

    RCLCPP_DEBUG(m_node->get_logger(), "Linear actuator manual command: %.3f m", position_m);
}
