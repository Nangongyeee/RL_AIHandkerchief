# Piper RL Deploy 使用说明

## 概述

`piper_rl_deploy` 是一个专门用于在 Piper 机器人上部署强化学习模型的 ROS 2 包。它支持 PyTorch (.pt) 和 ONNX (.onnx) 两种模型格式，提供了完整的实时控制框架。

## 功能特性

- 🤖 **实时模型推理**: 支持 PyTorch 和 ONNX 模型
- 📊 **观测历史**: 可配置的历史缓存系统
- 🛡️ **安全保护**: 内置安全检查和紧急停止
- ⚙️ **灵活配置**: YAML 配置文件支持
- 🔄 **多频率控制**: 独立的控制和推理频率

## 快速开始

### 1. 准备模型文件

将你训练好的模型文件放置到 `models/` 目录：

```bash
# 复制 PyTorch 模型
cp /path/to/your/model.pt src/piper_rl_deploy/models/

# 或复制 ONNX 模型  
cp /path/to/your/model.onnx src/piper_rl_deploy/models/
```

### 2. 配置参数

编辑 `config/piper_rl_config.yaml`：

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

## 详细配置

### 模型配置

```yaml
model_path: "models/piper_policy.pt"  # 模型文件路径
model_type: "pytorch"                 # "pytorch" 或 "onnx"
use_history: true                     # 是否使用观测历史
obs_dim: 48                          # 观测维度
action_dim: 12                       # 动作维度
history_length: 50                   # 历史长度
```

### 控制配置

```yaml
control_frequency: 200.0    # 控制循环频率 (Hz)
inference_frequency: 50.0   # 模型推理频率 (Hz)
```

### 关节配置

```yaml
joint_names: [...]          # 关节名称列表
default_kp: [...]          # PD控制器比例增益
default_kd: [...]          # PD控制器微分增益
action_scale: [...]        # 动作缩放因子
joint_pos_offset: [...]    # 关节位置偏移
```

## ROS 接口

### 订阅话题

- `/cmd_vel` (geometry_msgs/Twist): 速度命令
- `/joint_states` (sensor_msgs/JointState): 关节状态
- `/imu` (sensor_msgs/Imu): IMU 数据

### 发布话题

- `/joint_command` (sensor_msgs/JointState): 关节控制命令
- `/rl_actions` (std_msgs/Float32MultiArray): 原始RL动作
- `/piper_status` (piper_msgs/PiperStatusMsg): 系统状态

## 启动参数

```bash
# 基本启动
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py

# 指定模型
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py \
    model_path:=models/my_model.pt \
    model_type:=pytorch

# 使用自定义配置
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py \
    config_file:=config/my_config.yaml
```

## 安全机制

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

```bash
# 发布紧急停止信号
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"
```

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

控制器会自动订阅 `/cmd_vel` 话题，可以与导航栈无缝集成。

## 许可证

Apache 2.0 License
