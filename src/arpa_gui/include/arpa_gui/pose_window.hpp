#ifndef __ARPA_GUI_POSE_WINDOW_HPP__
#define __ARPA_GUI_POSE_WINDOW_HPP__

#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QSlider>
#include <QLabel>
#include <QGroupBox>
#include <QTimer>
#include <QProgressBar>
#include <QTextEdit>
#include <QCheckBox>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "std_srvs/srv/trigger.hpp"
#include "ur_manipulation/srv/plan_to_pose.hpp"
#include "ur_manipulation/srv/execute_plan.hpp"
#include "ur_manipulation/srv/stop_motion.hpp"

class PoseWindow : public QWidget
{
    Q_OBJECT
public:
    PoseWindow(rclcpp::Node::SharedPtr node);
    ~PoseWindow() override = default;

private slots:
    void planPose();
    void updateDepth();
    void executePlan();
    void stopMotion();
    void onSourceFrameChanged(const QString &frame);
    void onTargetFrameChanged(const QString &frame);
    void goHome();
    void updateCurrentPose();
    void onPrismaticChanged(int value);
    void testMoveUp();

private:
    void setupUI();
    void setupConnections();
    void applyStylesheet();
    void populateFrameList();
    void onFrameChanged();
    void logStatus(const QString &message, bool isError = false);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS node
    rclcpp::Node::SharedPtr m_node;

    // Poses
    geometry_msgs::msg::Pose m_home_pose;

    // ============ CURRENT POSE DISPLAY ============
    QLabel *m_current_x, *m_current_y, *m_current_z;
    QLabel *m_current_roll, *m_current_pitch, *m_current_yaw;
    QGroupBox *m_current_pose_group;

    // ============ JOINT STATES DISPLAY ============
    QLabel *m_joint_labels[6];
    QProgressBar *m_joint_bars[6];
    QGroupBox *m_joint_states_group;
    std::vector<double> m_current_joint_values;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_sub;

    // ============ TARGET POSE INPUT ============
    QLineEdit *m_x, *m_y, *m_z, *m_roll, *m_pitch, *m_yaw;
    QLineEdit *m_tx, *m_ty, *m_tz, *m_tr, *m_tp, *m_tyw;
    QGroupBox *m_target_pose_group;

    // ============ FRAME SELECTORS ============
    QComboBox *m_source_frame_selector;
    QComboBox *m_target_frame_selector;

    // ============ PRISMATIC CONTROL ============
    QSlider *m_prismatic_slider;
    QLineEdit *m_prismatic_value;
    QGroupBox *m_prismatic_group;

    // ============ CONTROL BUTTONS ============
    QPushButton *m_plan_btn;
    QPushButton *m_exec_btn;
    QPushButton *m_stop_btn;
    QPushButton *m_home_btn;
    QPushButton *m_update_depth_btn;
    QPushButton *m_test_btn;  // Temporary test button
    QCheckBox *m_cartesian_checkbox;  // Enable straight-line Cartesian motion

    // ============ STATUS LOG ============
    QTextEdit *m_status_log;
    QGroupBox *m_status_group;

    // Frame tracking
    std::string m_source_frame;
    std::string m_target_frame;

    // TF
    tf2_ros::Buffer m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    // Timer for periodic updates
    QTimer *m_update_timer;

    // ROS2 clients and publishers
    rclcpp::Client<ur_manipulation::srv::PlanToPose>::SharedPtr m_plan_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_update_depth_client;
    rclcpp::Client<ur_manipulation::srv::ExecutePlan>::SharedPtr m_exec_client;
    rclcpp::Client<ur_manipulation::srv::StopMotion>::SharedPtr m_stop_client;
    // Linear actuator controller - dual mode: manual slider + MoveIt 7-DOF planning
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_linear_actuator_pub;
};

#endif // __ARPA_GUI_POSE_WINDOW_HPP__
