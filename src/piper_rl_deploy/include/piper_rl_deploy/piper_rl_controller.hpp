#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "model_loader.hpp"

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

namespace piper_rl_deploy {

struct RobotObservation {
    std::vector<float> joint_positions;
    std::vector<float> joint_velocities;
    std::vector<float> joint_efforts;
    std::vector<float> imu_orientation;  // quaternion [w, x, y, z]
    std::vector<float> imu_angular_velocity;
    std::vector<float> imu_linear_acceleration;
    std::vector<float> base_lin_vel;
    std::vector<float> base_ang_vel;
    std::vector<float> commands;  // [vx, vy, vyaw]
    std::vector<float> actions_history;
};

struct RobotCommand {
    std::vector<float> joint_positions;
    std::vector<float> joint_velocities;
    std::vector<float> joint_efforts;
    std::vector<float> joint_kp;
    std::vector<float> joint_kd;
};

class PiperRLController : public rclcpp::Node {
public:
    explicit PiperRLController(const std::string& node_name = "piper_rl_controller");
    ~PiperRLController() = default;

private:
    // ROS 订阅和发布
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr inference_timer_;
    
    // 模型加载器
    std::unique_ptr<ModelLoader> model_loader_;
    
    // 机器人状态
    RobotObservation current_obs_;
    RobotCommand current_cmd_;
    
    // 历史缓存
    HistoryBuffer<std::vector<float>> obs_history_;
    HistoryBuffer<std::vector<float>> action_history_;
    
    // 控制参数
    double control_frequency_;
    double inference_frequency_;
    size_t obs_dim_;
    size_t action_dim_;
    size_t history_length_;
    
    // 模型参数
    std::string model_path_;
    ModelType model_type_;
    bool use_history_;
    
    // 关节配置
    std::vector<std::string> joint_names_;
    std::vector<double> default_kp_;
    std::vector<double> default_kd_;
    std::vector<double> action_scale_;
    std::vector<double> joint_pos_offset_;
    
    // 命令缓存
    geometry_msgs::msg::Twist cmd_vel_;
    bool cmd_vel_received_;
    
    // 状态标志
    bool model_ready_;
    bool robot_ready_;
    bool emergency_stop_;
    
    // 回调函数
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // 控制循环
    void controlLoop();
    void inferenceLoop();
    
    // 工具函数
    void loadParameters();
    void initializeModel();
    void initializeRobot();
    
    std::vector<float> computeObservation();
    std::vector<float> processActions(const std::vector<float>& raw_actions);
    void publishJointCommands(const std::vector<float>& actions);
    void publishStatus();
    
    // 安全检查
    bool safetyCheck(const std::vector<float>& actions);
    void emergencyStop();
    
    // 数据处理
    std::vector<float> quaternionToEuler(const std::vector<float>& quat);
    std::vector<float> normalizeObservation(const std::vector<float>& obs);
    std::vector<float> clipActions(const std::vector<float>& actions);
};

} // namespace piper_rl_deploy
