# RL_AIHandkerchief

一个基于 ROS 2 Humble 的 Piper 机器人强化学习项目。

## 项目概述

本项目包含 Piper 机器人的 ROS 2 包，支持仿真、描述文件、消息定义和 MoveIt 配置。项目基于 ROS 2 Humble 和 Isaac Lab 环境。

## 项目结构

```
RL_AIHandkerchief/
├── src/piper_utils/           # Piper 机器人相关包
│   ├── piper/                 # 核心 Piper 包
│   ├── piper_description/     # 机器人描述文件 (URDF/Xacro)
│   ├── piper_humble/          # Humble 特定配置
│   ├── piper_moveit/          # MoveIt 配置
│   ├── piper_msgs/            # 自定义消息定义
│   └── piper_sim/             # 仿真相关文件
├── build/                     # 构建输出目录
├── install/                   # 安装目录
├── log/                       # 构建日志
└── setup_env.sh              # 环境配置脚本
```

## 环境要求

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10
- Conda/Miniconda
- Isaac Lab 环境

## 快速开始

### 1. 一键环境配置（推荐）

```bash
conda activate env_isaaclab
./setup_env.sh  # 自动配置所有环境并构建项目
```

### 2. 快速启动

```bash
./quick_start.sh          # 设置环境
./quick_start.sh display  # 启动机器人显示
./quick_start.sh moveit   # 启动 MoveIt
```

### 3. 使用 Makefile（可选）

```bash
make help      # 查看所有可用命令
make setup     # 完整环境设置
make build     # 构建项目
make check     # 环境检查
```

### 4. 手动配置

```bash
conda activate env_isaaclab
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 工具脚本说明

| 脚本 | 功能 | 使用场景 |
|-----|------|---------|
| `setup_env.sh` | 完整环境配置和构建 | 首次设置或重置环境 |
| `quick_start.sh` | 快速启动和演示 | 日常开发使用 |
| `check_env.sh` | 环境状态检查 | 问题诊断 |
| `Makefile` | 项目管理快捷命令 | 简化常用操作 |

## VS Code 配置

项目包含 `RL_AIHandkerchief.code-workspace` 配置文件：

```bash
# 使用方法
code RL_AIHandkerchief.code-workspace
```

**预配置功能:**
- Python 解释器路径（env_isaaclab）
- ROS 2 文件关联（.launch, .urdf, .msg 等）
- 构建文件夹排除
- C++ 和 Python 代码补全
- 推荐扩展列表

## 包说明

- **piper_msgs**: 自定义消息定义
- **piper_description**: 机器人模型描述 (URDF/Xacro)
- **piper_gazebo/piper_mujoco**: 仿真配置
- **piper_*_moveit**: MoveIt 运动规划配置

## 常见问题

如果构建失败，运行 `./check_env.sh` 检查环境状态。

**常见依赖问题:**
```bash
# 自动修复（推荐）
./setup_env.sh

# 手动安装
pip install catkin_pkg empy==3.3.4 lark
```

## 基本使用

```bash
# 显示机器人模型
ros2 launch piper_description display.launch.py

# 启动 MoveIt 运动规划
ros2 launch piper_moveit demo.launch.py
```

---

**项目链接:** https://github.com/Nangongyeee/RL_AIHandkerchief


## vicon_brigde_ros2

````bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
cd cyclonedds-cxx
mkdir build
cd build
cmake ..
make -j
````