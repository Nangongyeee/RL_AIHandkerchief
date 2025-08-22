# Piper机械臂MIT模式力控制

本文档介绍如何使用新增的MIT（Model-based Impedance Tuning）模式来实现机械臂的高力度控制，特别适用于投掷等需要爆发力的动作。

## 功能概述

### 新增功能
1. **MIT模式控制**: 通过`JointMitCtrl`函数实现关节级的力控制
2. **增强PosCmd**: 扩展原有位置命令，支持MIT模式参数
3. **力度倍数控制**: 可调节的力度放大系数
4. **自定义PD参数**: 针对投掷动作优化的刚度和阻尼设置

### 新增消息类型
- `JointMitCmd.msg`: 单个关节的MIT控制命令
- 扩展的`PosCmd.msg`: 包含MIT模式参数的位置命令

## 使用方法

### 1. 启动MIT模式控制节点

```bash
# 方法1: 使用专用launch文件 (推荐)
ros2 launch piper piper_mit_control.launch.py

# 方法2: 使用参数启动
ros2 run piper piper_ctrl_single_node --ros-args \
  -p enable_mit_mode:=true \
  -p mit_force_multiplier:=2.0 \
  -p can_port:=can0 \
  -p gripper_exist:=true
```

### 2. 快速测试MIT模式

```bash
# 运行测试脚本
ros2 run piper test_mit_control

# 或者运行简化投掷器
ros2 run piper simple_mit_thrower
```

### 3. 编程使用MIT模式

#### 3.1 通过增强PosCmd使用

```python
from piper_msgs.msg import PosCmd

# 创建位置命令
pos_cmd = PosCmd()

# 基本位置参数
pos_cmd.x = 0.3
pos_cmd.y = 0.0
pos_cmd.z = 0.4
pos_cmd.roll = 0.0
pos_cmd.pitch = 0.3  # 投掷角度
pos_cmd.yaw = 0.0

# 启用MIT模式
pos_cmd.use_mit_mode = True
pos_cmd.force_multiplier = 2.5  # 力度倍数

# 自定义PD参数
pos_cmd.joint_kp = [200.0, 180.0, 160.0, 120.0, 80.0, 60.0]  # 高刚度
pos_cmd.joint_kd = [12.0, 10.0, 8.0, 6.0, 4.0, 3.0]        # 适中阻尼

# 高力矩设置 - 投掷的核心
pos_cmd.joint_torques = [35.0, 45.0, 30.0, 20.0, 15.0, 10.0]  # 高力矩

# 发布命令
pub.publish(pos_cmd)
```

#### 3.2 通过单独MIT命令使用

```python
from piper_msgs.msg import JointMitCmd

# 对关节2进行高力控制 (主要投掷关节)
mit_cmd = JointMitCmd()
mit_cmd.motor_num = 2
mit_cmd.pos_ref = 1.2      # 目标位置 (弧度)
mit_cmd.vel_ref = 0.5      # 目标速度 (rad/s)
mit_cmd.kp = 200.0         # 高刚度
mit_cmd.kd = 10.0          # 适中阻尼
mit_cmd.t_ref = 45.0       # 高力矩 (Nm)

# 发布MIT命令
mit_pub.publish(mit_cmd)
```

## 参数调节指南

### 力度相关参数

| 参数 | 范围 | 说明 | 投掷推荐值 |
|------|------|------|------------|
| `force_multiplier` | 1.0-5.0 | 总体力度倍数 | 2.0-3.0 |
| `joint_kp` | 50-500 | 关节刚度 | 100-300 |
| `joint_kd` | 1-20 | 关节阻尼 | 5-15 |
| `joint_torques` | 5-50 | 关节力矩 (Nm) | 10-45 |

### 投掷优化建议

1. **关节2 (主投掷关节)**: 使用最高kp和torque值
2. **关节1 (基座)**: 高刚度保证稳定性
3. **关节3**: 配合关节2的辅助投掷
4. **手腕关节 (4-6)**: 较低参数避免震荡

### 安全注意事项

⚠️ **重要安全提醒**:
- MIT模式会显著增加机械臂输出力度
- 首次使用时请降低参数值进行测试
- 确保机械臂周围有足够安全空间
- 投掷物体重量不应超过机械臂负载能力
- 建议在专业人员指导下使用

## 故障排除

### 常见问题

1. **MIT模式无效果**
   - 检查`enable_mit_mode`参数是否为true
   - 确认piper SDK版本支持`JointMitCtrl`函数
   - 查看日志中的MIT控制确认信息

2. **力度不够**
   - 增加`force_multiplier`值
   - 提高`joint_torques`参数
   - 检查关节是否达到力矩限制

3. **动作不稳定**
   - 降低`joint_kp`值
   - 增加`joint_kd`阻尼
   - 减小力度倍数

### 日志检查

启用MIT模式后，注意查看以下日志信息：
```
[INFO] enable_mit_mode is True
[INFO] Using MIT mode for enhanced force control
[INFO] MIT Control Joint1: kp=200.0, kd=10.0, torque=35.0
```

## 技术原理

MIT模式通过以下方式增强机械臂力度：

1. **直接力矩控制**: 绕过标准位置控制器，直接设定关节输出力矩
2. **高响应PD控制**: 优化的刚度和阻尼参数提供更快响应
3. **动态参数调节**: 根据任务需求实时调整控制参数
4. **协调多关节**: 各关节间的力矩分配优化

这种方法特别适合需要爆发力的任务，如投掷、击打、快速抓取等。

## 扩展应用

除了投掷，MIT模式还可用于：
- 快速抓取重物
- 高速装配作业  
- 冲击性操作
- 力控插装
- 动态平衡控制

## 联系支持

如果在使用过程中遇到问题，请：
1. 检查机械臂硬件连接
2. 确认CAN通信正常
3. 查看系统日志信息
4. 参考piper SDK官方文档
