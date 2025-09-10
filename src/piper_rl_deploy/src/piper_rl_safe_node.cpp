#include "piper_rl_deploy/piper_rl_safe_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("piper_rl_safe_node"), "Received signal %d, shutting down...", signum);
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // 创建节点
        auto node = std::make_shared<piper_rl_deploy::PiperRLSafeController>("piper_rl_safe_node");
        
        RCLCPP_INFO(node->get_logger(), "Starting Piper RL Safe deployment node...");
        RCLCPP_INFO(node->get_logger(), "This node directly subscribes to coordinate_test published topics:");
        RCLCPP_INFO(node->get_logger(), "  - /handkerchief_piperroot (handkerchief position in Piper_root frame)");
        RCLCPP_INFO(node->get_logger(), "  - /handkerchief_velocity (handkerchief velocity in Piper_root frame)");
        RCLCPP_INFO(node->get_logger(), "  - /end_effector_piperroot (end effector position in Piper_root frame)");
        RCLCPP_INFO(node->get_logger(), "  - /joint_states_single (current joint states)");
        
        // 运行节点
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("piper_rl_safe_node"), "Exception: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("piper_rl_safe_node"), "Piper RL Safe node shutdown complete");
    rclcpp::shutdown();
    return 0;
}
