#include "piper_rl_deploy/piper_rl_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("piper_rl_node"), "Received signal %d, shutting down...", signum);
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
        auto node = std::make_shared<piper_rl_deploy::PiperRLController>("piper_rl_node");
        
        RCLCPP_INFO(node->get_logger(), "Starting Piper RL deployment node...");
        
        // 运行节点
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("piper_rl_node"), "Exception: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("piper_rl_node"), "Piper RL node shutdown complete");
    rclcpp::shutdown();
    return 0;
}
