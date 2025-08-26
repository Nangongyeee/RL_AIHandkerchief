# Piper RL Deploy 使用说明

## 概述

`piper_rl_deploy` 是一个用于在 Piper 机器人上部署强化学习模型的 ROS 2 包。当前实现支持 PyTorch (.pt) 和 ONNX (.onnx) 模型格式（通过统一的 `ModelLoader`），并提供周期性控制循环与推理循环。

## 功能特性

-- 🤖 **实时模型推理**: PyTorch / ONNX 模型加载与推理
-- � **双循环架构**: 控制频率与推理频率解耦
-- �️ **可选历史缓存**: 通过 `use_history` 与 `history_length` 控制（当前默认关闭）
-- ⚙️ **参数化配置**: 统一通过 YAML 文件设置（启动文件不再单独传话题名）
-- �️ **基础安全检查**: 动作范围与状态就绪检查（可扩展）

## 快速开始

### 1. 准备模型文件

将你训练好的模型文件放置到 `models/` 目录：

```bash
# 复制 PyTorch 模型
cp /path/to/your/model.pt src/piper_rl_deploy/models/

# 或复制 ONNX 模型  
cp /path/to/your/model.onnx src/piper_rl_deploy/models/
```

### 2. 配置参数（仅通过 YAML）

编辑 `config/piper_rl_config.yaml`（启动文件不再传递话题名，请在此修改）：

```yaml
# 模型配置
model_path: "models/your_model.pt"
model_type: "pytorch"  # 或 "onnx"
obs_dim: 48           # 根据你的模型调整
action_dim: 12        # 根据你的机器人调整

# 关节配置
joint_names: ["joint_1", "joint_2", ...]  # 实际关节名称
```

### 3. 构建和运行

```bash
# 构建包
colcon build --packages-select piper_rl_deploy

# 启动部署
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py
```

## 参数说明

### 模型/历史参数

```yaml
model_path: "models/piper_policy.pt"  # 模型文件路径
model_type: "pytorch"                 # "pytorch" 或 "onnx"
use_history: true                     # 是否使用观测历史
obs_dim: 48                          # 观测维度
action_dim: 12                       # 动作维度
history_length: 50                   # 历史长度
```

### 控制循环参数

```yaml
control_frequency: 200.0    # 控制循环频率 (Hz)
inference_frequency: 50.0   # 模型推理频率 (Hz)
```

### 关节与动作参数

```yaml
joint_names: [...]          # 关节名称列表
default_kp: [...]          # PD控制器比例增益
default_kd: [...]          # PD控制器微分增益
action_scale: [...]        # 动作缩放因子
joint_pos_offset: [...]    # 关节位置偏移
```

## ROS 接口（当前实现）

### 订阅话题

- `/joint_states_single` (sensor_msgs/JointState): Piper 硬件/驱动输出的关节状态
- `<robot_base_pose_topic>` (geometry_msgs/PoseStamped): 机械臂底座世界位姿（由 YAML 中 `robot_base_pose_topic` 指定，示例：`/vicon/root0822/root0822`）
- `<handkerchief_pose_topic>` (geometry_msgs/PoseStamped): 目标物（手绢/物体）世界位姿（由 YAML 中 `handkerchief_pose_topic` 指定）

> 说明：当前代码未订阅 `/cmd_vel` 与 `/imu`，如需集成导航或 IMU，请扩展源码后再更新本节。

### 发布话题

- `/joint_states` (sensor_msgs/JointState): 关节控制命令（控制目标位姿）
- `/rl_actions` (std_msgs/Float32MultiArray): 模型推理输出的原始动作（供调试/记录）
- `/piper_status` (std_msgs/String): JSON 格式状态信息（包含模型/机器人就绪、紧急停止标志等）

## 启动与自定义

```bash
# 基本启动
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py

# 指定模型（覆盖 YAML 中 model_path / model_type）
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py \
    model_path:=models/my_model.pt \
    model_type:=pytorch

# 指定自定义配置文件（包含所有话题与参数）
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py \
    config_file:=config/my_config.yaml

> 话题名称请在 YAML 中修改，不再通过 launch 参数传递。
```

## 安全机制（当前实现）

### 自动安全检查

- 动作值范围检查 (NaN, inf, 超限检测)
- 关节位置和速度限制
- 力矩限制

### 紧急停止

系统检测到异常时会自动触发紧急停止：
- 发布零速度命令
- 停止模型推理
- 记录错误日志

### 手动紧急停止

当前版本未实现订阅 `/emergency_stop` 话题（README 旧说明已移除）。如需要：
1. 在控制器中添加 `rclcpp::Subscription<std_msgs::msg::Bool>`
2. 回调内设置 `emergency_stop_ = true;`
3. 更新本节示例

或直接在代码中调用 `emergencyStop()` 扩展接口。

## 故障排除

### 模型加载失败

1. 检查模型文件路径是否正确
2. 确认模型类型设置正确
3. 验证 PyTorch/ONNX 运行时是否安装

### 关节控制异常

1. 检查关节名称映射是否正确
2. 验证关节状态话题是否发布
3. 确认PD增益设置合理

### 性能问题

1. 调整控制和推理频率
2. 检查模型推理时间
3. 考虑使用ONNX模型优化性能

## 开发指南

### 添加新的观测

在 `computeObservation()` 函数中添加：

```cpp
// 添加新的观测
obs.insert(obs.end(), new_observation.begin(), new_observation.end());
```

### 自定义动作处理

在 `processActions()` 函数中修改：

```cpp
// 自定义动作处理逻辑
for (size_t i = 0; i < actions.size(); ++i) {
    actions[i] = customProcessing(actions[i], i);
}
```

### 扩展安全检查

在 `safetyCheck()` 函数中添加：

```cpp
// 添加自定义安全检查
if (customSafetyCondition(actions)) {
    return false;
}
```

## 性能优化建议

1. **模型优化**: 使用 ONNX 格式可以提升推理速度
2. **频率调整**: 根据硬件性能调整控制和推理频率
3. **历史长度**: 减少历史长度可以降低内存使用
4. **线程设置**: 调整 PyTorch 线程数量

## 与其他包的集成

### 与 Piper 描述包集成

```bash
# 同时启动描述和控制
ros2 launch piper_description robot.launch.py &
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py
```

### 与导航系统集成

（当前未订阅 `/cmd_vel`，需要导航控制可自行添加订阅并在 `computeObservation()` 中融合）

## 许可证

Apache 2.0 License
