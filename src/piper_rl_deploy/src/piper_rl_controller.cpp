#include "piper_rl_deploy/piper_rl_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

#define M_PI 3.14159265358979323846

namespace piper_rl_deploy {

PiperRLController::PiperRLController(const std::string& node_name)
    : Node(node_name)
    , obs_history_(50)  // 默认保存50个历史观测
    , action_history_(10)  // 默认保存10个历史动作
    , model_ready_(false)
    , robot_ready_(false)
    , emergency_stop_(false)
{
    // 加载参数
    loadParameters();
    
    // 初始化模型
    initializeModel();
    
    // 初始化ROS接口
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_single", 10,
        std::bind(&PiperRLController::jointStateCallback, this, std::placeholders::_1)
    );
    
    // 订阅机械臂底座位置（使用参数化话题名）
    robot_base_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        robot_base_pose_topic_, 10,
        std::bind(&PiperRLController::robotBasePoseCallback, this, std::placeholders::_1)
    );
    
    // 订阅手绢位置（使用参数化话题名）
    handkerchief_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        handkerchief_pose_topic_, 10,
        std::bind(&PiperRLController::handkerchiefPoseCallback, this, std::placeholders::_1)
    );

    // 发布关节控制命令到piper节点期望的话题
    joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    action_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rl_actions", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/piper_status", 10);
    
    // 创建定时器
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / control_frequency_),
        std::bind(&PiperRLController::controlLoop, this)
    );
    
    inference_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / inference_frequency_),
        std::bind(&PiperRLController::inferenceLoop, this)
    );
    
    // 初始化机器人
    initializeRobot();
    
    RCLCPP_INFO(this->get_logger(), "Piper RL Controller initialized");
}

PiperRLController::~PiperRLController() {
    // 优雅关闭
    emergency_stop_ = true;
    
    // 停止定时器
    if (control_timer_) {
        control_timer_->cancel();
    }
    if (inference_timer_) {
        inference_timer_->cancel();
    }
    
    // 发布零位置命令
    if (joint_cmd_pub_ && robot_ready_) {
        std::vector<float> zero_actions(action_dim_, 0.0);
        publishJointCommands(zero_actions);
    }
    
    RCLCPP_INFO(this->get_logger(), "Piper RL Controller shutting down");
}

void PiperRLController::loadParameters() {
    // 声明参数
    this->declare_parameter("control_frequency", 200.0);
    this->declare_parameter("inference_frequency", 50.0);
    this->declare_parameter("model_path", "");
    this->declare_parameter("model_type", "pytorch");
    this->declare_parameter("use_history", false);    // 训练代码没有使用历史
    this->declare_parameter("obs_dim", 12);           // 6(关节角度) + 3(手绢位置) + 3(手绢速度) 
    this->declare_parameter("action_dim", 6);         // 6个关节
    this->declare_parameter("history_length", 1);     // 不使用历史
    
    // 话题名称参数
    this->declare_parameter("robot_base_pose_topic", "/robot_base_pose");
    this->declare_parameter("handkerchief_pose_topic", "/handkerchief_pose");
    
    // 获取参数
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    inference_frequency_ = this->get_parameter("inference_frequency").as_double();
    model_path_ = this->get_parameter("model_path").as_string();
    use_history_ = this->get_parameter("use_history").as_bool();
    obs_dim_ = this->get_parameter("obs_dim").as_int();
    action_dim_ = this->get_parameter("action_dim").as_int();
    history_length_ = this->get_parameter("history_length").as_int();
    
    // 话题名称
    robot_base_pose_topic_ = this->get_parameter("robot_base_pose_topic").as_string();
    handkerchief_pose_topic_ = this->get_parameter("handkerchief_pose_topic").as_string();
    
    std::string model_type_str = this->get_parameter("model_type").as_string();
    if (model_type_str == "pytorch") {
        model_type_ = ModelType::PYTORCH;
    } else if (model_type_str == "onnx") {
        model_type_ = ModelType::ONNX;
    } else {
        model_type_ = ModelType::NONE;
        RCLCPP_ERROR(this->get_logger(), "Unknown model type: %s", model_type_str.c_str());
    }
    
    // 关节配置参数 - 匹配piper节点
    this->declare_parameter("joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    this->declare_parameter("default_kp", std::vector<double>{80.0, 80.0, 80.0, 80.0, 80.0, 80.0});
    this->declare_parameter("default_kd", std::vector<double>{4.0, 4.0, 4.0, 4.0, 4.0, 4.0});
    this->declare_parameter("action_scale", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter("joint_pos_offset", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    default_kp_ = this->get_parameter("default_kp").as_double_array();
    default_kd_ = this->get_parameter("default_kd").as_double_array();
    action_scale_ = this->get_parameter("action_scale").as_double_array();
    joint_pos_offset_ = this->get_parameter("joint_pos_offset").as_double_array();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully");
}

void PiperRLController::initializeModel() {
    if (model_path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Model path not specified! Running without model inference.");
        model_ready_ = false;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loading model from path: %s", model_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Expected input dim: %zu, output dim: %zu", obs_dim_, action_dim_);
    
    model_loader_ = std::make_unique<ModelLoader>(model_path_, model_type_);
    
    if (model_loader_->loadModel()) {
        model_loader_->setInputDim(obs_dim_);
        model_loader_->setOutputDim(action_dim_);
        model_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully from: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Model ready for inference with input_dim=%zu, output_dim=%zu", 
                    obs_dim_, action_dim_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load model from: %s", model_path_.c_str());
        model_ready_ = false;
    }
}

void PiperRLController::initializeRobot() {
    // 初始化机器人状态
    current_obs_.joint_positions.resize(joint_names_.size(), 0.0);
    current_obs_.joint_velocities.resize(joint_names_.size(), 0.0);
    current_obs_.joint_efforts.resize(joint_names_.size(), 0.0);
    current_obs_.actions_history.resize(action_dim_, 0.0);
    
    // 初始化位置信息
    current_obs_.handkerchief_position.resize(3, 0.0);
    current_obs_.handkerchief_velocity.resize(3, 0.0);
    current_obs_.robot_base_position.resize(3, 0.0);
    current_obs_.robot_base_orientation.resize(4, 0.0);
    current_obs_.robot_base_orientation[3] = 1.0; // 初始化为单位四元数 [0,0,0,1]
    current_obs_.handkerchief_world_position.resize(3, 0.0);
    current_obs_.handkerchief_world_orientation.resize(4, 0.0);
    current_obs_.handkerchief_world_orientation[3] = 1.0; // 初始化为单位四元数 [0,0,0,1]
    
    // 初始化控制指令
    current_cmd_.joint_positions.resize(joint_names_.size(), 0.0);
    current_cmd_.joint_velocities.resize(joint_names_.size(), 0.0);
    current_cmd_.joint_efforts.resize(joint_names_.size(), 0.0);
    current_cmd_.joint_kp.resize(joint_names_.size());
    current_cmd_.joint_kd.resize(joint_names_.size());
    
    // 设置默认增益
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        current_cmd_.joint_kp[i] = i < default_kp_.size() ? default_kp_[i] : 100.0;
        current_cmd_.joint_kd[i] = i < default_kd_.size() ? default_kd_[i] : 5.0;
    }
    
    robot_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Robot initialized with %zu joints", joint_names_.size());
}

void PiperRLController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 根据关节名称映射更新状态
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it != msg->name.end()) {
            size_t idx = std::distance(msg->name.begin(), it);
            
            if (idx < msg->position.size()) {
                current_obs_.joint_positions[i] = static_cast<float>(msg->position[idx]);
            }
            if (idx < msg->velocity.size()) {
                current_obs_.joint_velocities[i] = static_cast<float>(msg->velocity[idx]);
            }
            if (idx < msg->effort.size()) {
                current_obs_.joint_efforts[i] = static_cast<float>(msg->effort[idx]);
            }
        }
    }
}

void PiperRLController::robotBasePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 更新机械臂底座在世界坐标系下的位置和姿态
    current_obs_.robot_base_position[0] = static_cast<float>(msg->pose.position.x);
    current_obs_.robot_base_position[1] = static_cast<float>(msg->pose.position.y);
    current_obs_.robot_base_position[2] = static_cast<float>(msg->pose.position.z);
    
    current_obs_.robot_base_orientation[0] = static_cast<float>(msg->pose.orientation.x);
    current_obs_.robot_base_orientation[1] = static_cast<float>(msg->pose.orientation.y);
    current_obs_.robot_base_orientation[2] = static_cast<float>(msg->pose.orientation.z);
    current_obs_.robot_base_orientation[3] = static_cast<float>(msg->pose.orientation.w);
    
    // 重新计算手绢在机械臂底座坐标系下的位置
    current_obs_.handkerchief_position = computeRelativePosition(
        current_obs_.handkerchief_world_position,
        current_obs_.robot_base_position,
        current_obs_.robot_base_orientation
    );
    
    RCLCPP_DEBUG(this->get_logger(), "Robot base pose updated: [%.3f, %.3f, %.3f]", 
                 current_obs_.robot_base_position[0], 
                 current_obs_.robot_base_position[1], 
                 current_obs_.robot_base_position[2]);
}

void PiperRLController::handkerchiefPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 保存上一次的位置用于计算速度
    static std::vector<float> prev_position = {0.0f, 0.0f, 0.0f};
    static auto prev_time = this->now();
    
    // 更新手绢在世界坐标系下的位置和姿态
    current_obs_.handkerchief_world_position[0] = static_cast<float>(msg->pose.position.x);
    current_obs_.handkerchief_world_position[1] = static_cast<float>(msg->pose.position.y);
    current_obs_.handkerchief_world_position[2] = static_cast<float>(msg->pose.position.z);
    
    current_obs_.handkerchief_world_orientation[0] = static_cast<float>(msg->pose.orientation.x);
    current_obs_.handkerchief_world_orientation[1] = static_cast<float>(msg->pose.orientation.y);
    current_obs_.handkerchief_world_orientation[2] = static_cast<float>(msg->pose.orientation.z);
    current_obs_.handkerchief_world_orientation[3] = static_cast<float>(msg->pose.orientation.w);
    
    // 计算手绢在机械臂底座坐标系下的位置
    current_obs_.handkerchief_position = computeRelativePosition(
        current_obs_.handkerchief_world_position,
        current_obs_.robot_base_position,
        current_obs_.robot_base_orientation
    );
    
    // 计算速度 (简单的数值微分)
    auto current_time = this->now();
    double dt = (current_time - prev_time).seconds();
    
    if (dt > 0.001) { // 避免除以很小的数
        for (int i = 0; i < 3; ++i) {
            current_obs_.handkerchief_velocity[i] = 
                (current_obs_.handkerchief_position[i] - prev_position[i]) / static_cast<float>(dt);
        }
        
        prev_position = current_obs_.handkerchief_position;
        prev_time = current_time;
    }
    
    RCLCPP_INFO(this->get_logger(), "Handkerchief pose updated - World: [%.3f, %.3f, %.3f], Relative: [%.3f, %.3f, %.3f]", 
                 current_obs_.handkerchief_world_position[0], 
                 current_obs_.handkerchief_world_position[1], 
                 current_obs_.handkerchief_world_position[2],
                 current_obs_.handkerchief_position[0],
                 current_obs_.handkerchief_position[1],
                 current_obs_.handkerchief_position[2]);
}

void PiperRLController::controlLoop() {
    if (!robot_ready_ || emergency_stop_) {
        return;
    }
    
    // 发布状态信息
    publishStatus();
}

void PiperRLController::inferenceLoop() {
    if (!model_ready_ || !robot_ready_ || emergency_stop_) {
        if (!model_ready_) {
            RCLCPP_DEBUG(this->get_logger(), "Model not ready for inference");
        }
        if (!robot_ready_) {
            RCLCPP_DEBUG(this->get_logger(), "Robot not ready for inference");
        }
        return;
    }
    
    // 计算观测
    std::vector<float> observation = computeObservation();
    
    RCLCPP_INFO(this->get_logger(), "Computing observation, size: %zu", observation.size());
    RCLCPP_INFO(this->get_logger(), "Joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                observation[0], observation[1], observation[2], 
                observation[3], observation[4], observation[5]);
    
    // 如果使用历史，添加到历史缓存
    if (use_history_) {
        obs_history_.push(observation);
        
        // 构建历史观测向量
        auto history = obs_history_.getBuffer();
        if (history.size() < history_length_) {
            return; // 等待足够的历史数据
        }
        
        // 将历史数据拼接
        std::vector<float> full_obs;
        for (const auto& obs : history) {
            full_obs.insert(full_obs.end(), obs.begin(), obs.end());
        }
        observation = full_obs;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting model inference with observation size: %zu", observation.size());
    
    // 模型推理
    std::vector<float> raw_actions = model_loader_->inference(observation);
    
    if (raw_actions.empty()) {
        RCLCPP_WARN(this->get_logger(), "Model inference failed - empty output");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Model inference successful, output size: %zu", raw_actions.size());
    RCLCPP_INFO(this->get_logger(), "Raw actions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                raw_actions[0], raw_actions[1], raw_actions[2], 
                raw_actions[3], raw_actions[4], raw_actions[5]);
    
    // 处理动作
    std::vector<float> processed_actions = processActions(raw_actions);
    
    RCLCPP_INFO(this->get_logger(), "Processed actions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                processed_actions[0], processed_actions[1], processed_actions[2], 
                processed_actions[3], processed_actions[4], processed_actions[5]);
    
    // 安全检查
    if (!safetyCheck(processed_actions)) {
        RCLCPP_WARN(this->get_logger(), "Safety check failed, stopping");
        emergencyStop();
        return;
    }
    
    // 发布关节命令
    publishJointCommands(processed_actions);
    
    // 更新动作历史
    action_history_.push(processed_actions);
    current_obs_.actions_history = processed_actions;
    
    RCLCPP_INFO(this->get_logger(), "Inference loop completed successfully");
}

std::vector<float> PiperRLController::computeObservation() {
    std::vector<float> obs;
    
    // 根据您的描述，观测包括：
    // 1. 机械臂的6轴角度 (6维)
    // 2. 手绢在机械臂底座坐标系下的位置 (3维)
    // 3. 手绢在机械臂底座坐标系下的速度 (3维)
    // 总共12维观测
    
    // 1. 机械臂6轴角度
    obs.insert(obs.end(), current_obs_.joint_positions.begin(), current_obs_.joint_positions.end());
    
    // 2. 手绢在机械臂底座坐标系下的位置 (3维)
    obs.insert(obs.end(), current_obs_.handkerchief_position.begin(), current_obs_.handkerchief_position.end());
    
    // 3. 手绢在机械臂底座坐标系下的速度 (3维)
    obs.insert(obs.end(), current_obs_.handkerchief_velocity.begin(), current_obs_.handkerchief_velocity.end());
    
    RCLCPP_DEBUG(this->get_logger(), "Observation computed - Joint pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f], "
                 "Handkerchief pos: [%.3f, %.3f, %.3f], vel: [%.3f, %.3f, %.3f]",
                 obs[0], obs[1], obs[2], obs[3], obs[4], obs[5],
                 obs[6], obs[7], obs[8], obs[9], obs[10], obs[11]);
    
    return obs;
}

std::vector<float> PiperRLController::processActions(const std::vector<float>& raw_actions) {
    std::vector<float> actions = raw_actions;
    
    // 裁剪动作到 [-1, 1] 范围
    actions = clipActions(actions);
    
    // 根据训练代码，将 [-1,1] 的动作转换为实际关节位置
    // target_positions = 0.5 * (actions + 1) * (upper_limits - lower_limits) + lower_limits
    
    // Piper机械臂的关节限制 (根据URDF和训练代码推断)
    std::vector<float> lower_limits = {-3.14f, -1.57f, -3.14f, -1.57f, -3.14f, -1.57f};
    std::vector<float> upper_limits = {3.14f, 1.57f, 3.14f, 1.57f, 3.14f, 1.57f};
    
    // 转换动作到实际关节位置
    for (size_t i = 0; i < actions.size() && i < 6; ++i) {
        float range = upper_limits[i] - lower_limits[i];
        actions[i] = 0.5f * (actions[i] + 1.0f) * range + lower_limits[i];
    }
    
    return actions;
}

void PiperRLController::publishJointCommands(const std::vector<float>& actions) {
    sensor_msgs::msg::JointState joint_cmd;
    joint_cmd.header.stamp = this->now();
    joint_cmd.name = joint_names_;
    
    joint_cmd.position.resize(actions.size());
    joint_cmd.velocity.resize(actions.size());
    joint_cmd.effort.resize(actions.size());
    
    for (size_t i = 0; i < actions.size(); ++i) {
        joint_cmd.position[i] = actions[i];
        joint_cmd.velocity[i] = 0.0;  // 速度由控制器计算
        joint_cmd.effort[i] = 0.0;   // 力矩由控制器计算
    }
    
    joint_cmd_pub_->publish(joint_cmd);
    
    // 发布原始动作用于调试
    std_msgs::msg::Float32MultiArray action_msg;
    action_msg.data = actions;
    action_pub_->publish(action_msg);
}

void PiperRLController::publishStatus() {
    std_msgs::msg::String status;
    
    // 创建状态JSON字符串
    std::string status_json = "{";
    status_json += "\"model_ready\":" + std::string(model_ready_ ? "true" : "false") + ",";
    status_json += "\"robot_ready\":" + std::string(robot_ready_ ? "true" : "false") + ",";
    status_json += "\"emergency_stop\":" + std::string(emergency_stop_ ? "true" : "false") + ",";
    status_json += "\"timestamp\":" + std::to_string(this->now().seconds());
    status_json += "}";
    
    status.data = status_json;
    status_pub_->publish(status);
}

bool PiperRLController::safetyCheck(const std::vector<float>& actions) {
    // 检查动作是否在合理范围内
    for (const auto& action : actions) {
        if (std::isnan(action) || std::isinf(action)) {
            return false;
        }
        if (std::abs(action) > 10.0) {  // 设置一个合理的上限
            return false;
        }
    }
    return true;
}

void PiperRLController::emergencyStop() {
    emergency_stop_ = true;
    
    // 发布零速度命令
    std::vector<float> zero_actions(action_dim_, 0.0);
    publishJointCommands(zero_actions);
    
    RCLCPP_ERROR(this->get_logger(), "Emergency stop activated!");
}

std::vector<float> PiperRLController::normalizeObservation(const std::vector<float>& obs) {
    // 这里可以实现观测归一化逻辑
    // 目前直接返回原始观测
    return obs;
}

std::vector<float> PiperRLController::clipActions(const std::vector<float>& actions) {
    std::vector<float> clipped = actions;
    
    // 裁剪到 [-1, 1] 范围
    for (auto& action : clipped) {
        action = std::max(-1.0f, std::min(1.0f, action));
    }
    
    return clipped;
}

std::vector<float> PiperRLController::computeRelativePosition(const std::vector<float>& world_pos, 
                                                             const std::vector<float>& base_pos, 
                                                             const std::vector<float>& base_quat) {
    std::vector<float> relative_pos(3, 0.0f);
    
    // 检查输入向量的大小
    if (world_pos.size() < 3 || base_pos.size() < 3 || base_quat.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Invalid input sizes for coordinate transformation");
        return relative_pos;
    }
    
    // 计算世界坐标到底座坐标的平移
    float dx = world_pos[0] - base_pos[0];
    float dy = world_pos[1] - base_pos[1];
    float dz = world_pos[2] - base_pos[2];
    
    // 从四元数创建旋转矩阵（底座坐标系到世界坐标系的旋转）
    // 我们需要的是逆变换（世界坐标系到底座坐标系）
    float qx = base_quat[0];
    float qy = base_quat[1];
    float qz = base_quat[2];
    float qw = base_quat[3];
    
    // 归一化四元数
    float norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm > 0.0001f) {
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;
    } else {
        // 默认为单位四元数
        qx = qy = qz = 0.0f;
        qw = 1.0f;
    }
    
    // 旋转矩阵的逆（转置），将世界坐标转换到底座坐标
    float r11 = 1.0f - 2.0f*(qy*qy + qz*qz);
    float r12 = 2.0f*(qx*qy + qw*qz);
    float r13 = 2.0f*(qx*qz - qw*qy);
    
    float r21 = 2.0f*(qx*qy - qw*qz);
    float r22 = 1.0f - 2.0f*(qx*qx + qz*qz);
    float r23 = 2.0f*(qy*qz + qw*qx);
    
    float r31 = 2.0f*(qx*qz + qw*qy);
    float r32 = 2.0f*(qy*qz - qw*qx);
    float r33 = 1.0f - 2.0f*(qx*qx + qy*qy);
    
    // 应用逆旋转变换
    relative_pos[0] = r11*dx + r21*dy + r31*dz;
    relative_pos[1] = r12*dx + r22*dy + r32*dz;
    relative_pos[2] = r13*dx + r23*dy + r33*dz;
    
    return relative_pos;
}

} // namespace piper_rl_deploy
