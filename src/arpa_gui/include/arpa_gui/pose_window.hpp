#ifndef __ARPA_GUI_POSE_WINDOW_HPP__
#define __ARPA_GUI_POSE_WINDOW_HPP__
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QComboBox>
#include <QFormLayout>
#include <QSlider>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
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
    
    private:
    void populateFrameList();
    void onFrameChanged();
    void onPrismaticChanged(int value);

    rclcpp::Node::SharedPtr m_node;
    geometry_msgs::msg::Pose m_home_pose;
    QLineEdit *m_x, *m_y, *m_z, *m_roll, *m_pitch, *m_yaw;
    QLineEdit *m_tx, *m_ty, *m_tz, *m_tr, *m_tp, *m_tyw;
    QComboBox *m_source_frame_selector;
    QComboBox *m_target_frame_selector;
    QSlider *m_prismatic_slider;
    QLineEdit *m_prismatic_value;
    std::string m_source_frame;
    std::string m_target_frame;

    tf2_ros::Buffer m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    rclcpp::Client<ur_manipulation::srv::PlanToPose>::SharedPtr m_plan_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_update_depth_client;
    rclcpp::Client<ur_manipulation::srv::ExecutePlan>::SharedPtr m_exec_client;
    rclcpp::Client<ur_manipulation::srv::StopMotion>::SharedPtr m_stop_client;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_linear_actuator_pub;
};
#endif // __ARPA_GUI_POSE_WINDOW_HPP__