#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    std::vector<float> actions_history;
    
    // 位置信息
    std::vector<float> handkerchief_position;     // 手绢在机械臂底座坐标系下的位置 (x, y, z)
    std::vector<float> handkerchief_velocity;     // 手绢在机械臂底座坐标系下的速度 (vx, vy, vz)
    std::vector<float> robot_base_position;       // 机械臂底座在世界坐标系下的位置
    std::vector<float> robot_base_orientation;    // 机械臂底座在世界坐标系下的姿态(四元数)
    std::vector<float> handkerchief_world_position; // 手绢在世界坐标系下的位置
    std::vector<float> handkerchief_world_orientation; // 手绢在世界坐标系下的姿态(四元数)
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
    ~PiperRLController();

private:
    // ROS 订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_base_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr handkerchief_pose_sub_;
    
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
    
    // 话题名称
    std::string robot_base_pose_topic_;
    std::string handkerchief_pose_topic_;
    
    // 机械偏置参数
    float handkerchief_z_offset_;
    
    // 关节配置
    std::vector<std::string> joint_names_;
    std::vector<double> default_kp_;
    std::vector<double> default_kd_;
    std::vector<double> action_scale_;
    std::vector<double> joint_pos_offset_;
    
    // 状态标志
    bool model_ready_;
    bool robot_ready_;
    bool emergency_stop_;
    
    // 回调函数
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void robotBasePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void handkerchiefPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
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
    
    // 坐标变换
    std::vector<float> computeRelativePosition(const std::vector<float>& world_pos, 
                                             const std::vector<float>& base_pos, 
                                             const std::vector<float>& base_quat);
};

} // namespace piper_rl_deploy
