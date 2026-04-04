#include "arpa_gui/pose_window.hpp"
#include <QDateTime>
#include <QScrollBar>
#include <cmath>
#include <random>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdlib>

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
      m_tf_buffer(node->get_clock()),
      m_sequence_running(false)
{
    // Initialize TF listener after buffer is constructed
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(m_tf_buffer);
    setWindowTitle("ARPA Robot Control Panel");
    setMinimumSize(800, 800);  // Wider to accommodate right panel

    m_current_joint_values.resize(6, 0.0);

    setupUI();
    setupConnections();
    applyStylesheet();

    // ROS2 clients
    m_plan_client = m_node->create_client<arpa_control::srv::PlanToPose>("plan_to_pose");
    m_plan_to_joint_client = m_node->create_client<arpa_control::srv::PlanToJoint>("plan_to_joint");
    m_plan_linear_actuator_client = m_node->create_client<arpa_control::srv::PlanLinearActuator>("plan_linear_actuator");
    m_update_depth_client = m_node->create_client<std_srvs::srv::Trigger>("update_depth");
    m_exec_client = m_node->create_client<arpa_control::srv::ExecutePlan>("execute_plan");
    m_stop_client = m_node->create_client<arpa_control::srv::StopMotion>("stop_motion");
    // Linear actuator controller - supports both manual slider control and MoveIt 7-DOF planning
    m_linear_actuator_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>("/linear_actuator_controller/commands", 10);

    // Humanoid teleport publishers
    m_humanoid_teleport_pub = m_node->create_publisher<geometry_msgs::msg::Point>("/humanoid/teleport_delta", 10);
    m_humanoid_pose_pub = m_node->create_publisher<geometry_msgs::msg::Pose>("/humanoid/teleport_pose", 10);

    // Subscribe to joint states
    m_joint_state_sub = m_node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&PoseWindow::jointStateCallback, this, std::placeholders::_1));

    // Subscribe to BT status and feedback
    m_bt_status_sub = m_node->create_subscription<std_msgs::msg::String>(
        "/bt_status", 10,
        std::bind(&PoseWindow::btStatusCallback, this, std::placeholders::_1));
    m_bt_feedback_sub = m_node->create_subscription<std_msgs::msg::String>(
        "/bt_feedback", 10,
        std::bind(&PoseWindow::btFeedbackCallback, this, std::placeholders::_1));

    // Client for screw sequence service
    m_run_screw_sequence_client = m_node->create_client<std_srvs::srv::Trigger>("run_screw_sequence");

    // Parameter client for bt_executor_node (to set transfer_strategy)
    m_bt_param_client = std::make_shared<rclcpp::AsyncParametersClient>(m_node, "bt_executor_node");

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
    logBtStatus("BT Status Monitor initialized - waiting for updates...");
}

void PoseWindow::setupUI()
{
    // ============ MAIN HORIZONTAL SPLIT LAYOUT ============
    auto *mainHLayout = new QHBoxLayout;
    mainHLayout->setSpacing(10);
    mainHLayout->setContentsMargins(10, 10, 10, 10);

    // ============ LEFT PANEL (existing controls in scroll area) ============
    auto *leftWidget = new QWidget;
    auto *leftLayout = new QVBoxLayout(leftWidget);
    leftLayout->setSpacing(10);
    leftLayout->setContentsMargins(5, 5, 5, 5);

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
    leftLayout->addWidget(m_current_pose_group);

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
    leftLayout->addWidget(m_joint_states_group);

    // ============ PRISMATIC CONTROL GROUP ============
    m_prismatic_group = new QGroupBox("Linear Actuator Control");
    auto *prismaticOuterLayout = new QVBoxLayout;
    auto *prismaticSliderLayout = new QHBoxLayout;

    m_prismatic_slider = new QSlider(Qt::Horizontal);
    m_prismatic_slider->setMinimum(100);   // 0.1m = 100mm
    m_prismatic_slider->setMaximum(2000);  // 2.0m = 2000mm
    m_prismatic_slider->setValue(100);
    m_prismatic_slider->setTickInterval(100);
    m_prismatic_slider->setTickPosition(QSlider::TicksBelow);

    m_prismatic_value = new QLineEdit("0.100");
    m_prismatic_value->setReadOnly(true);
    m_prismatic_value->setFixedWidth(80);
    m_prismatic_value->setAlignment(Qt::AlignRight);

    prismaticSliderLayout->addWidget(new QLabel("Extension:"));
    prismaticSliderLayout->addWidget(m_prismatic_slider, 1);
    prismaticSliderLayout->addWidget(m_prismatic_value);
    prismaticSliderLayout->addWidget(new QLabel("m"));

    m_plan_linear_actuator_btn = new QPushButton("Plan Linear Actuator");
    m_plan_linear_actuator_btn->setMinimumHeight(40);

    prismaticOuterLayout->addLayout(prismaticSliderLayout);
    prismaticOuterLayout->addWidget(m_plan_linear_actuator_btn);

    m_prismatic_group->setLayout(prismaticOuterLayout);
    leftLayout->addWidget(m_prismatic_group);

    // ============ TARGET POSE GROUP ============
    m_target_pose_group = new QGroupBox("Relative Motion (Deltas from Current Pose)");
    auto *targetLayout = new QFormLayout;

    // Frame selectors
    auto *frameLayout = new QHBoxLayout;
    m_source_frame_selector = new QComboBox;
    m_target_frame_selector = new QComboBox;
    m_source_frame_selector->addItems({"base_link", "tool0", "floor_link", "pelvis"});
    m_target_frame_selector->addItems({"base_link", "tool0", "floor_link", "pelvis"});
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
    leftLayout->addWidget(m_target_pose_group);

    // ============ CONTROL BUTTONS ============
    auto *buttonGroup = new QGroupBox("Control");
    auto *buttonLayout = new QGridLayout;

    m_plan_btn = new QPushButton("Plan");
    m_exec_btn = new QPushButton("Execute");
    m_stop_btn = new QPushButton("STOP");
    m_home_btn = new QPushButton("Home");
    m_update_depth_btn = new QPushButton("Update Depth");
    m_test_btn = new QPushButton("TEST: Move 1cm Up");
    m_goto_screw1_btn = new QPushButton("Goto Screw 1");
    m_cartesian_checkbox = new QCheckBox("Cartesian (straight-line)");
    m_cartesian_checkbox->setChecked(true);  // Default to Cartesian for smoother motion
    m_cartesian_checkbox->setToolTip("Use straight-line path planning instead of sampling-based (RRTConnect)");

    m_plan_btn->setMinimumHeight(40);
    m_exec_btn->setMinimumHeight(40);
    m_stop_btn->setMinimumHeight(50);
    m_home_btn->setMinimumHeight(40);
    m_update_depth_btn->setMinimumHeight(40);
    m_test_btn->setMinimumHeight(40);
    m_goto_screw1_btn->setMinimumHeight(40);

    buttonLayout->addWidget(m_cartesian_checkbox, 0, 0, 1, 2);  // Span 2 columns
    buttonLayout->addWidget(m_plan_btn, 1, 0);
    buttonLayout->addWidget(m_exec_btn, 1, 1);
    buttonLayout->addWidget(m_home_btn, 2, 0);
    buttonLayout->addWidget(m_update_depth_btn, 2, 1);
    buttonLayout->addWidget(m_test_btn, 3, 0, 1, 2);
    buttonLayout->addWidget(m_goto_screw1_btn, 4, 0, 1, 2);
    buttonLayout->addWidget(m_stop_btn, 5, 0, 1, 2);

    buttonGroup->setLayout(buttonLayout);
    leftLayout->addWidget(buttonGroup);

    // ============ STATUS LOG ============
    m_status_group = new QGroupBox("Status Log");
    auto *statusLayout = new QVBoxLayout;

    m_status_log = new QTextEdit;
    m_status_log->setReadOnly(true);
    m_status_log->setMaximumHeight(120);
    m_status_log->setLineWrapMode(QTextEdit::WidgetWidth);

    statusLayout->addWidget(m_status_log);
    m_status_group->setLayout(statusLayout);
    leftLayout->addWidget(m_status_group);

    // ============ HUMANOID CONTROL GROUP ============
    m_humanoid_group = new QGroupBox("Humanoid (H12 / pelvis)");
    auto *humanoidLayout = new QVBoxLayout;

    // Current humanoid position label
    m_humanoid_pos_label = new QLabel("Humanoid (pelvis): unknown");
    humanoidLayout->addWidget(m_humanoid_pos_label);

    // Teleport delta controls
    auto *teleportLayout = new QHBoxLayout;
    teleportLayout->addWidget(new QLabel("ΔX:"));
    m_humanoid_dx = new QLineEdit("0.0");
    m_humanoid_dx->setFixedWidth(60);
    teleportLayout->addWidget(m_humanoid_dx);
    teleportLayout->addWidget(new QLabel("ΔY:"));
    m_humanoid_dy = new QLineEdit("0.0");
    m_humanoid_dy->setFixedWidth(60);
    teleportLayout->addWidget(m_humanoid_dy);
    teleportLayout->addWidget(new QLabel("ΔZ:"));
    m_humanoid_dz = new QLineEdit("0.0");
    m_humanoid_dz->setFixedWidth(60);
    teleportLayout->addWidget(m_humanoid_dz);
    humanoidLayout->addLayout(teleportLayout);

    auto *teleportBtnLayout = new QHBoxLayout;
    m_humanoid_teleport_btn = new QPushButton("Teleport humanoid");
    m_humanoid_teleport_btn->setMinimumHeight(35);
    m_humanoid_random_btn = new QPushButton("Random humanoid pose");
    m_humanoid_random_btn->setMinimumHeight(35);
    teleportBtnLayout->addWidget(m_humanoid_teleport_btn);
    teleportBtnLayout->addWidget(m_humanoid_random_btn);
    humanoidLayout->addLayout(teleportBtnLayout);

    // Move-away controls
    auto *moveAwayLayout = new QHBoxLayout;
    moveAwayLayout->addWidget(new QLabel("Move farther from pelvis by (m):"));
    m_humanoid_offset_dist = new QLineEdit("0.30");
    m_humanoid_offset_dist->setFixedWidth(60);
    moveAwayLayout->addWidget(m_humanoid_offset_dist);
    humanoidLayout->addLayout(moveAwayLayout);

    m_humanoid_move_away_btn = new QPushButton("Plan: Move EE away from humanoid");
    m_humanoid_move_away_btn->setMinimumHeight(40);
    humanoidLayout->addWidget(m_humanoid_move_away_btn);

    m_humanoid_group->setLayout(humanoidLayout);
    leftLayout->addWidget(m_humanoid_group);

    leftLayout->addStretch();

    // Put left content in scroll area
    auto *leftScrollArea = new QScrollArea;
    leftScrollArea->setWidget(leftWidget);
    leftScrollArea->setWidgetResizable(true);
    leftScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    leftScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    leftScrollArea->setMinimumWidth(480);

    mainHLayout->addWidget(leftScrollArea, 1);  // stretch factor 1

    // ============ RIGHT PANEL (BT Status Monitor) ============
    auto *rightWidget = new QWidget;
    auto *rightLayout = new QVBoxLayout(rightWidget);
    rightLayout->setSpacing(10);
    rightLayout->setContentsMargins(5, 5, 5, 5);

    // Transfer strategy selector
    auto *strategyLayout = new QHBoxLayout;
    strategyLayout->addWidget(new QLabel("Transfer Strategy:"));
    m_strategy_selector = new QComboBox;
    m_strategy_selector->addItem("Linear Actuator (fast)", "linear_actuator");
    m_strategy_selector->addItem("Constrained Box (experimental)", "constrained");
    m_strategy_selector->setCurrentIndex(1);  // default: constrained (straight lateral transfer, fewer collisions)
    m_strategy_selector->setToolTip("How the robot moves between screw locations in XY");
    strategyLayout->addWidget(m_strategy_selector, 1);
    rightLayout->addLayout(strategyLayout);

    // Create Sequence button
    m_create_sequence_btn = new QPushButton("Create Sequence");
    m_create_sequence_btn->setMinimumHeight(50);
    m_create_sequence_btn->setToolTip("Constrained drop-down: 3 cm above -> 3 cm down -> 4 s wait -> 3 cm up -> constrained transfer to next screw. Straight Cartesian motions, collision-checked against gantry/structures.");
    rightLayout->addWidget(m_create_sequence_btn);

    // BT Status Monitor group
    m_bt_status_group = new QGroupBox("BT Status Monitor");
    auto *btStatusLayout = new QVBoxLayout;

    m_bt_status_monitor = new QTextEdit;
    m_bt_status_monitor->setReadOnly(true);
    m_bt_status_monitor->setLineWrapMode(QTextEdit::WidgetWidth);

    btStatusLayout->addWidget(m_bt_status_monitor);
    m_bt_status_group->setLayout(btStatusLayout);
    rightLayout->addWidget(m_bt_status_group, 1);  // stretch factor 1 to fill space

    rightWidget->setFixedWidth(300);
    mainHLayout->addWidget(rightWidget, 0);  // stretch factor 0 (fixed width)

    setLayout(mainHLayout);
}

void PoseWindow::setupConnections()
{
    connect(m_plan_btn, &QPushButton::clicked, this, &PoseWindow::planPose);
    connect(m_plan_linear_actuator_btn, &QPushButton::clicked, this, &PoseWindow::planLinearActuator);
    connect(m_exec_btn, &QPushButton::clicked, this, &PoseWindow::executePlan);
    connect(m_stop_btn, &QPushButton::clicked, this, &PoseWindow::stopMotion);
    connect(m_home_btn, &QPushButton::clicked, this, &PoseWindow::goHome);
    connect(m_update_depth_btn, &QPushButton::clicked, this, &PoseWindow::updateDepth);
    connect(m_test_btn, &QPushButton::clicked, this, &PoseWindow::testMoveUp);
    connect(m_goto_screw1_btn, &QPushButton::clicked, this, &PoseWindow::goto_screw1);
    connect(m_create_sequence_btn, &QPushButton::clicked, this, &PoseWindow::onCreateSequenceClicked);

    connect(m_humanoid_teleport_btn, &QPushButton::clicked, this, &PoseWindow::humanoidTeleport);
    connect(m_humanoid_random_btn, &QPushButton::clicked, this, &PoseWindow::humanoidRandomPose);
    connect(m_humanoid_move_away_btn, &QPushButton::clicked, this, &PoseWindow::humanoidMoveAway);

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

    // Update humanoid position display
    try {
        auto tf_g1 = m_tf_buffer.lookupTransform("floor_link", "pelvis", tf2::TimePointZero);
        const auto &g = tf_g1.transform.translation;
        m_humanoid_pos_label->setText(QString("Humanoid pelvis: (%1, %2, %3)")
            .arg(g.x, 0, 'f', 3).arg(g.y, 0, 'f', 3).arg(g.z, 0, 'f', 3));
    } catch (const tf2::TransformException &) {
        m_humanoid_pos_label->setText("Humanoid (pelvis): not available");
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
    // req->use_cartesian = m_cartesian_checkbox->isChecked();
    // logStatus(QString("Planning mode: %1").arg(req->use_cartesian ? "Cartesian (straight-line)" : "Sampling-based (RRTConnect)"));

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

void PoseWindow::planLinearActuator()
{
    // Get linear actuator target from slider (joint limits 0.1 to 2.0m = 100 to 2000mm)
    double linear_actuator_target = static_cast<double>(m_prismatic_slider->value()) / 1000.0;
    logStatus(QString("Planning linear actuator to: %1 m").arg(linear_actuator_target, 0, 'f', 3));

    auto req = std::make_shared<arpa_control::srv::PlanLinearActuator::Request>();
    req->position = linear_actuator_target;

    m_plan_linear_actuator_client->async_send_request(req,
        [this](rclcpp::Client<arpa_control::srv::PlanLinearActuator>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                QMetaObject::invokeMethod(this, [this]() {
                    logStatus("Linear actuator planning successful!");
                });
            } else {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("Linear actuator planning failed: " + QString::fromStdString(result->message), true);
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

void PoseWindow::goto_screw1()
{
    // Screw 1 joint positions from Screw Locations.yaml (at screw, gantry kept at current)
    // Order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
    auto req = std::make_shared<arpa_control::srv::PlanToJoint::Request>();
    req->joint1 = -1.2103412787066858;  // shoulder_pan
    req->joint2 = -1.8353263340392054;  // shoulder_lift
    req->joint3 = -1.4010802507400513;  // elbow
    req->joint4 = -1.4565215867808838;  // wrist_1
    req->joint5 = 4.704919815063477;    // wrist_2
    req->joint6 = -1.8898323217975062;  // wrist_3

    logStatus("Goto Screw 1: planning to screw 1 joint position (plan_to_joint)...");

    m_plan_to_joint_client->async_send_request(req,
        [this](rclcpp::Client<arpa_control::srv::PlanToJoint>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                QMetaObject::invokeMethod(this, [this]() {
                    logStatus("Goto Screw 1: planning OK, executing...");
                    executePlan();
                });
            } else {
                QMetaObject::invokeMethod(this, [this, result]() {
                    logStatus("Goto Screw 1 failed: " + QString::fromStdString(result->message), true);
                });
            }
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
    // req->use_cartesian = true;  // Always use Cartesian for small test movements

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
    // request->use_cartesian = false;  // Use sampling-based for large home movements

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
    // Convert slider value to position (joint limits 0.1 to 2.0m = 100 to 2000mm)
    double position_m = static_cast<double>(value) / 1000.0;
    m_prismatic_value->setText(QString::number(position_m, 'f', 3));

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.push_back(position_m);
    m_linear_actuator_pub->publish(cmd);

    RCLCPP_DEBUG(m_node->get_logger(), "Linear actuator manual command: %.3f m", position_m);
}

void PoseWindow::logBtStatus(const QString &message, bool isError)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString coloredMsg;

    if (isError) {
        coloredMsg = QString("<span style='color: #e74c3c;'>[%1] %2</span>").arg(timestamp, message);
    } else if (message.contains("SUCCESS")) {
        coloredMsg = QString("<span style='color: #2ecc71;'>[%1] %2</span>").arg(timestamp, message);
    } else if (message.contains("RUNNING")) {
        coloredMsg = QString("<span style='color: #3498db;'>[%1] %2</span>").arg(timestamp, message);
    } else if (message.contains("FAILURE")) {
        coloredMsg = QString("<span style='color: #e74c3c;'>[%1] %2</span>").arg(timestamp, message);
    } else {
        coloredMsg = QString("<span style='color: #ecf0f1;'>[%1] %2</span>").arg(timestamp, message);
    }

    m_bt_status_monitor->append(coloredMsg);

    // Auto-scroll to bottom
    QScrollBar *sb = m_bt_status_monitor->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void PoseWindow::btStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    QString status = QString::fromStdString(msg->data);
    QMetaObject::invokeMethod(this, [this, status]() {
        logBtStatus(QString("BT Status: %1").arg(status));
    }, Qt::QueuedConnection);
}

void PoseWindow::btFeedbackCallback(const std_msgs::msg::String::SharedPtr msg)
{
    QString feedback = QString::fromStdString(msg->data);
    QMetaObject::invokeMethod(this, [this, feedback]() {
        logBtStatus(QString("BT Feedback: %1").arg(feedback));
    }, Qt::QueuedConnection);
}

void PoseWindow::onBtStatusReceived(const QString &status)
{
    logBtStatus(status);
}

void PoseWindow::onCreateSequenceClicked()
{
    if (m_sequence_running) {
        logStatus("Sequence already running", true);
        logBtStatus("Sequence already running - please wait", true);
        return;
    }

    if (!m_run_screw_sequence_client->wait_for_service(std::chrono::seconds(1))) {
        logStatus("run_screw_sequence service not available", true);
        logBtStatus("Service not available - is bt_executor_node running?", true);
        return;
    }

    m_sequence_running = true;
    m_create_sequence_btn->setEnabled(false);
    m_create_sequence_btn->setText("Running...");

    // Get selected strategy from dropdown
    QString strategy = m_strategy_selector->currentData().toString();
    // #region agent log
    {
        std::ostringstream o;
        o << "{\"hypothesisId\":\"H5\",\"location\":\"pose_window:onCreateSequenceClicked\",\"message\":\"gui_sends_strategy\",\"data\":{\"strategy\":\""
          << strategy.toStdString() << "\"}";
        o << ",\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count() << "}\n";
        const char* lp = std::getenv("DEBUG_LOG_PATH");
        std::string log_path = lp ? lp : "/root/ros2_ws/.cursor/debug.log";
        std::ofstream f(log_path, std::ios::app);
        if (f) f << o.str();
    }
    // #endregion
    logStatus(QString("Starting screw sequence with strategy: %1").arg(strategy));
    logBtStatus(QString("Starting screw sequence - strategy: %1").arg(m_strategy_selector->currentText()));

    // Set the transfer_strategy parameter on bt_executor_node before calling the service
    auto param = rclcpp::Parameter("transfer_strategy", strategy.toStdString());
    m_bt_param_client->set_parameters({param},
        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
            (void)future;  // We don't need to check the result strictly
            // Now call the service
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            m_run_screw_sequence_client->async_send_request(request,
                [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                    auto result = future.get();
                    QMetaObject::invokeMethod(this, [this, result]() {
                        m_sequence_running = false;
                        m_create_sequence_btn->setEnabled(true);
                        m_create_sequence_btn->setText("Create Sequence");

                        if (result->success) {
                            logStatus("Screw sequence completed successfully!");
                            logBtStatus("Sequence completed successfully!");
                        } else {
                            logStatus("Screw sequence failed: " + QString::fromStdString(result->message), true);
                            logBtStatus("Sequence failed: " + QString::fromStdString(result->message), true);
                        }
                    }, Qt::QueuedConnection);
                });
        });
}

void PoseWindow::humanoidTeleport()
{
    geometry_msgs::msg::Point delta;
    delta.x = m_humanoid_dx->text().toDouble();
    delta.y = m_humanoid_dy->text().toDouble();
    delta.z = m_humanoid_dz->text().toDouble();
    m_humanoid_teleport_pub->publish(delta);
    logStatus(QString("Teleporting humanoid by (%1, %2, %3)")
        .arg(delta.x, 0, 'f', 3).arg(delta.y, 0, 'f', 3).arg(delta.z, 0, 'f', 3));
}

void PoseWindow::humanoidMoveAway()
{
    double offset_m = m_humanoid_offset_dist->text().toDouble();
    if (offset_m <= 0.0) {
        logStatus("Offset must be positive", true);
        return;
    }

    try {
        auto tf_ee = m_tf_buffer.lookupTransform("floor_link", "tool0", tf2::TimePointZero);
        auto tf_g1 = m_tf_buffer.lookupTransform("floor_link", "pelvis", tf2::TimePointZero);

        double ee_x = tf_ee.transform.translation.x;
        double ee_y = tf_ee.transform.translation.y;
        double ee_z = tf_ee.transform.translation.z;
        double g1_x = tf_g1.transform.translation.x;
        double g1_y = tf_g1.transform.translation.y;
        double g1_z = tf_g1.transform.translation.z;

        double dx = ee_x - g1_x;
        double dy = ee_y - g1_y;
        double dz = ee_z - g1_z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < 1e-4) {
            logStatus("EE is at humanoid position, cannot compute direction", true);
            return;
        }

        double ux = dx / dist;
        double uy = dy / dist;
        double uz = dz / dist;

        double target_x = ee_x + ux * offset_m;
        double target_y = ee_y + uy * offset_m;
        double target_z = ee_z + uz * offset_m;

        logStatus(QString("Planning EE %1m farther from pelvis -> (%2, %3, %4) in floor_link")
            .arg(offset_m, 0, 'f', 2)
            .arg(target_x, 0, 'f', 3).arg(target_y, 0, 'f', 3).arg(target_z, 0, 'f', 3));

        auto req = std::make_shared<arpa_control::srv::PlanToPose::Request>();
        req->target_pose.header.frame_id = "floor_link";
        req->target_pose.pose.position.x = target_x;
        req->target_pose.pose.position.y = target_y;
        req->target_pose.pose.position.z = target_z;

        req->target_pose.pose.orientation = tf_ee.transform.rotation;

        m_plan_client->async_send_request(req,
            [this](rclcpp::Client<arpa_control::srv::PlanToPose>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                    QMetaObject::invokeMethod(this, [this]() {
                        logStatus("Move-away plan succeeded! Executing...");
                        executePlan();
                    }, Qt::QueuedConnection);
                } else {
                    QMetaObject::invokeMethod(this, [this, result]() {
                        logStatus("Move-away plan failed: " + QString::fromStdString(result->message), true);
                    }, Qt::QueuedConnection);
                }
            });
    } catch (const tf2::TransformException &ex) {
        logStatus(QString("TF lookup failed (is humanoid spawned?): %1").arg(ex.what()), true);
    }
}

void PoseWindow::humanoidRandomPose()
{
    // Workspace layout in floor_link (= world):
    //   Battery/table: X ~ [-0.87, 1.11], Y ~ [-0.74, 0.63], Z ~ 0.5m
    //   Left pillar + foot:  center (1.47, 0), foot 1.0x1.0m → X [0.97, 1.97], Y [-0.5, 0.5]
    //   Right pillar + foot: center (-1.47, 0), foot 1.0x1.0m → X [-1.97, -0.97], Y [-0.5, 0.5]
    //
    // Safe perimeter zones for the humanoid (standing on floor, pelvis Z=0.78):
    //   Front (+Y side): X [-1.0, 1.0], Y [1.0, 1.6]   — facing -Y toward table
    //   Back  (-Y side): X [-1.0, 1.0], Y [-1.6, -1.0]  — facing +Y toward table

    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> side_dist(0, 1);  // 0 = front, 1 = back
    std::uniform_real_distribution<double> x_dist(-1.0, 1.0);

    int side = side_dist(rng);
    double px = x_dist(rng);
    double py, yaw;

    if (side == 0) {
        // Front (+Y side)
        std::uniform_real_distribution<double> y_dist(1.0, 1.6);
        py = y_dist(rng);
        yaw = -M_PI / 2.0;  // facing -Y toward table
    } else {
        // Back (-Y side)
        std::uniform_real_distribution<double> y_dist(-1.6, -1.0);
        py = y_dist(rng);
        yaw = M_PI / 2.0;   // facing +Y toward table
    }

    double pz = 0.78;  // pelvis height (H12 / G1 similar)

    geometry_msgs::msg::Pose pose;
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);

    m_humanoid_pose_pub->publish(pose);

    QString side_str = (side == 0) ? "front (+Y)" : "back (-Y)";
    logStatus(QString("Random humanoid pose: %1 side -> (%2, %3, %4)")
        .arg(side_str)
        .arg(px, 0, 'f', 3).arg(py, 0, 'f', 3).arg(pz, 0, 'f', 3));
}
