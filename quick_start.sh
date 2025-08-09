#!/bin/bash

# 快速启动脚本 - RL_AIHandkerchief

set -e

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🤖 RL_AIHandkerchief 快速启动${NC}"
echo "=================================="

# 检查环境
if [[ "$CONDA_DEFAULT_ENV" != "env_isaaclab" ]]; then
    echo "❌ 请先激活 conda 环境:"
    echo "   conda activate env_isaaclab"
    exit 1
fi

# 设置 ROS 2 环境
if [[ -z "$ROS_DISTRO" ]]; then
    echo "🔧 设置 ROS 2 环境..."
    source /opt/ros/humble/setup.bash
fi

# 设置工作空间环境
if [[ -f "install/setup.bash" ]]; then
    echo "🔧 设置工作空间环境..."
    source install/setup.bash
else
    echo "⚠️  工作空间未构建，正在构建..."
    colcon build --symlink-install
    source install/setup.bash
fi

echo -e "${GREEN}✅ 环境已准备就绪！${NC}"
echo ""
echo "可用的启动选项："
echo "1. 显示机器人模型: ros2 launch piper_description display.launch.py"
echo "2. 启动 MoveIt: ros2 launch piper_moveit demo.launch.py"
echo "3. 查看可用包: ros2 pkg list | grep piper"
echo ""

# 如果有参数，执行相应命令
case "${1:-}" in
    "display")
        echo "🚀 启动机器人显示..."
        ros2 launch piper_description display.launch.py
        ;;
    "moveit")
        echo "🚀 启动 MoveIt..."
        ros2 launch piper_moveit demo.launch.py
        ;;
    "list")
        echo "📦 可用的 Piper 包："
        ros2 pkg list | grep piper
        ;;
    *)
        echo "💡 使用方法:"
        echo "   $0 [display|moveit|list]"
        ;;
esac
