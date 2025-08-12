#!/bin/zsh

# 快速启动脚本 - RL_AIHandkerchief

set -e

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🤖 RL_AIHandkerchief 快速启动${NC}"
echo "=================================="

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
if [[ "$CONDA_DEFAULT_ENV" != "env_isaaclab" ]]; then
    echo "🔧 激活 env_isaaclab 环境..."
    conda activate env_isaaclab
    if [[ "$?" -ne 0 ]]; then
        echo "❌ 无法激活 env_isaaclab 环境，请检查环境是否存在"
        exit 1
    fi
    # 确保激活成功
    eval "$(conda shell.zsh hook)"
    conda activate env_isaaclab
else
    echo "✅ 已在 env_isaaclab 环境中"
fi

echo "当前 conda 环境: $CONDA_DEFAULT_ENV"


# 设置 ROS 2 环境
if [[ -z "$ROS_DISTRO" ]]; then
    echo "🔧 设置 ROS 2 环境..."
    source /opt/ros/humble/setup.zsh
fi

# 设置工作空间环境
if [[ -f "install/setup.zsh" ]]; then
    echo "🔧 设置工作空间环境..."
    source install/setup.zsh
else
    echo "⚠️  工作空间未构建，正在构建..."
    colcon build --symlink-install
    source install/setup.zsh
fi

# 解决 libstdc++ 版本冲突 - 使用系统库而不是 conda 的
echo "🔧 解决 libstdc++ 版本冲突..."
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

echo -e "${GREEN}✅ 环境已准备就绪！${NC}"
echo ""
echo "可用的启动选项："
echo "1. 启动 Piper RL 部署: ros2 launch piper_rl_deploy piper_rl_deploy.launch.py"
echo "2. 启动单个 Piper: ros2 launch piper start_single_piper.launch.py"
echo ""

# 如果有参数，执行相应命令
case "${1:-}" in
    "rl" | "deploy")
        echo "🚀 启动 Piper RL 部署..."
        ros2 launch piper_rl_deploy piper_rl_deploy.launch.py
        ;;
    "single" | "piper")
        echo "🚀 启动单个 Piper..."
        ros2 launch piper start_single_piper.launch.py
        ;;
    *)
        echo "� 使用方法:"
        echo "   $0 [rl|deploy]  - 启动 Piper RL 部署"
        echo "   $0 [single|piper] - 启动单个 Piper"
        ;;
esac
