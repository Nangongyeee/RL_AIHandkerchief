#!/bin/bash

# Vicon Bridge 启动脚本
# 解决 conda 环境和 libstdc++ 版本冲突问题

echo "===== Vicon Bridge 启动脚本 ====="

# 初始化 conda
echo "🔧 初始化 conda 环境..."
if [ -f ~/miniconda3/etc/profile.d/conda.sh ]; then
    source ~/miniconda3/etc/profile.d/conda.sh
elif [ -f ~/anaconda3/etc/profile.d/conda.sh ]; then
    source ~/anaconda3/etc/profile.d/conda.sh
else
    echo "❌ 找不到 conda 安装，请确保 conda 已正确安装"
    exit 1
fi

# 激活 conda 环境
if [ "$CONDA_DEFAULT_ENV" != "env_isaaclab" ]; then
    echo "🔧 激活 env_isaaclab 环境..."
    conda activate env_isaaclab
    if [ "$?" -ne 0 ]; then
        echo "❌ 无法激活 env_isaaclab 环境，请检查环境是否存在"
        exit 1
    fi
else
    echo "✅ 已在 env_isaaclab 环境中"
fi
# 设置 ROS2 环境
echo "设置 ROS2 环境..."
source /opt/ros/humble/setup.bash
source ./install/setup.bash

# 解决 libstdc++ 版本冲突 - 使用系统库而不是 conda 的
echo "解决 libstdc++ 版本冲突..."
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# 设置 Vicon IP
export VICON_IP=${1:-192.168.10.1}
echo "Vicon IP: $VICON_IP"

echo "启动 Vicon Bridge..."
echo "================================"
ros2 launch vicon_bridge2 vicon.launch.py