# RL_AIHandkerchief

一个基于 ROS 2 Humble 的 Piper 机器人强化学习实机部署项目。

## 项目概述

本项目包含 Piper 机器人的 ROS 2 包，支持仿真、描述文件、消息定义和 MoveIt 配置。项目基于---

**项目链接:** https://github.com/Nangongyeee/RL_AIHandkerchief


## 克隆本项目

```bash
git clone https://github.com/Nangongyeee/RL_AIHandkerchief.git
```

## Vicon Bridge 配置和使用

### 安装依赖

1. **安装 CycloneDDS**:
```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
mkdir build && cd build
cmake ..
make -j
sudo make install
cd ~/RL_AIHandkerchief
```

2. **安装 CycloneDDS-CXX**:
```bash
cd ~
git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
cd cyclonedds-cxx
mkdir build && cd build
cmake ..
make -j
sudo make install
cd ~/RL_AIHandkerchief
```

## piper_sdk相关包

参考https://github.com/agilexrobotics/piper_ros.git的humble分支

## piper_rl_deploy配置

参考https://github.com/fan-ziqi/rl_sar.git


### 常见问题

**错误: `KeyError: 'VICON_IP'`**
- 原因: 缺少 VICON_IP 环境变量
- 解决: 设置 `export VICON_IP=你的Vicon系统IP` Isaac Lab 环境。

## 环境要求

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10
- Conda/Miniconda
- Isaac Lab 环境

## 快速开始

### 手动配置

```bash
source /opt/ros/humble/setup.bash && colcon build --symlink-install
source install/setup.bash
```

## 包说明

- **piper_msgs**: 自定义消息定义
- **piper_description**: 机器人模型描述 (URDF/Xacro)
- **piper_gazebo/piper_mujoco**: 仿真配置
- **piper_*_moveit**: MoveIt 运动规划配置
- **piper_rl_deploy**: 强化学习部署配置
- **vicon_bridge**: Vicon 数据桥接
- **vicon_msgs**: Vicon 消息定义


## 基本使用

**重要：保证你的电脑和Vicon主机、Vicon相机系统在同一局域网下**
![alt text](docs/image.png)
![alt text](docs/image2.png)
![alt text](docs/image3.png)
![alt text](docs/image4.png)

**重要：每次使用前必须先设置环境**

```bash

# 2. 设置 ROS 2 环境（根据你的 shell 选择）
source /opt/ros/humble/setup.bash  # bash 用户
source /opt/ros/humble/setup.zsh   # zsh 用户

# 3. 设置项目环境
source install/setup.bash  # bash 用户
source install/setup.zsh   # zsh 用户

# 启动单个 Piper（需要硬件连接）
ros2 launch piper start_single_piper.launch.py

# 测试机械臂连接是否正常
python test_mit_detailed.py

# 测试 Vicon 数据桥接
export VICON_IP=192.168.10.1
ros2 launch vicon_bridge start_vicon_bridge.launch.py

# 启动RL部署程序
ros2 launch piper_rl_deploy piper_rl_deploy.launch.py

```


---

**项目链接:** https://github.com/Nangongyeee/RL_AIHandkerchief


