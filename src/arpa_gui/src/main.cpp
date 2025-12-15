#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "arpa_gui/pose_window.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("arpa_gui");
    PoseWindow window(node);
    window.show();

    std::thread ros_thread([&]()
                           { rclcpp::spin(node); });
    int ret = app.exec();

    rclcpp::shutdown();
    ros_thread.join();
    return ret;
}