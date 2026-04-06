#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "arpa_gui/pose_window.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

// Global flag for clean shutdown
static std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signum) {
    g_shutdown_requested = true;
    QApplication::quit();
}

int main(int argc, char **argv)
{
    // Install signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        rclcpp::init(argc, argv);
        QApplication app(argc, argv);

        // Set application info
        app.setApplicationName("ARPA Control Panel");
        app.setOrganizationName("ARPA");

        auto node = std::make_shared<rclcpp::Node>("arpa_gui");

        PoseWindow window(node);
        window.show();

        // Run ROS2 spinning in a separate thread
        std::thread ros_thread([&node]() {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(node);
            while (rclcpp::ok() && !g_shutdown_requested) {
                executor.spin_some(std::chrono::milliseconds(10));
            }
        });

        // Run Qt event loop
        int ret = app.exec();

        // Cleanup
        g_shutdown_requested = true;
        rclcpp::shutdown();

        if (ros_thread.joinable()) {
            ros_thread.join();
        }

        return ret;

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}
