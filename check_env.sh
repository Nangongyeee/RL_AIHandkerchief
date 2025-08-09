#!/bin/bash

# 环境检查脚本 - RL_AIHandkerchief

echo "🔍 RL_AIHandkerchief 环境检查"
echo "============================="

# 检查函数
check_command() {
    if command -v "$1" &> /dev/null; then
        echo "✅ $1: $(which "$1")"
        if [[ -n "$2" ]]; then
            echo "   版本: $($1 --version 2>/dev/null | head -1 || echo "无法获取版本")"
        fi
    else
        echo "❌ $1: 未安装"
    fi
}

check_python_module() {
    if python3 -c "import $1" 2>/dev/null; then
        version=$(python3 -c "import $1; print(getattr($1, '__version__', '未知版本'))" 2>/dev/null)
        echo "✅ Python 模块 $1: $version"
    else
        echo "❌ Python 模块 $1: 未安装"
    fi
}

# 系统信息
echo ""
echo "📊 系统信息:"
echo "OS: $(lsb_release -d | cut -f2)"
echo "内核: $(uname -r)"
echo "Python: $(python3 --version)"

# Conda 环境
echo ""
echo "🐍 Conda 环境:"
if command -v conda &> /dev/null; then
    echo "✅ Conda: $(conda --version)"
    echo "当前环境: ${CONDA_DEFAULT_ENV:-无}"
    if [[ "$CONDA_DEFAULT_ENV" == "env_isaaclab" ]]; then
        echo "✅ env_isaaclab 环境已激活"
    else
        echo "⚠️  请激活 env_isaaclab 环境"
    fi
else
    echo "❌ Conda 未安装"
fi

# ROS 2 环境
echo ""
echo "🤖 ROS 2 环境:"
if [[ -n "$ROS_DISTRO" ]]; then
    echo "✅ ROS 发行版: $ROS_DISTRO"
    echo "ROS 路径: ${ROS_ROOT:-未设置}"
else
    echo "❌ ROS 2 环境未设置"
fi

# 工具检查
echo ""
echo "🛠️  工具检查:"
check_command "colcon" "version"
check_command "ros2" "version"
check_command "rviz2"
check_command "gazebo"

# Python 模块检查
echo ""
echo "🐍 Python 模块检查:"
check_python_module "catkin_pkg"
check_python_module "em"
check_python_module "lark"
check_python_module "rosbag2_py"

# 工作空间检查
echo ""
echo "📁 工作空间检查:"
if [[ -f "src/piper_utils/piper/package.xml" ]]; then
    echo "✅ Piper 包结构正确"
else
    echo "❌ Piper 包结构错误"
fi

if [[ -d "build" && -d "install" ]]; then
    echo "✅ 工作空间已构建"
    
    # 检查包是否正确安装
    if [[ -f "install/setup.bash" ]]; then
        source install/setup.bash 2>/dev/null
        echo "✅ 工作空间环境可用"
        
        # 列出已安装的包
        echo ""
        echo "📦 已安装的包:"
        if command -v ros2 &> /dev/null; then
            ros2 pkg list | grep piper | sed 's/^/   /'
        fi
    else
        echo "❌ 工作空间环境文件缺失"
    fi
else
    echo "⚠️  工作空间未构建，请运行 colcon build"
fi

# 网络检查（可选）
echo ""
echo "🌐 网络检查:"
if ping -c 1 github.com &> /dev/null; then
    echo "✅ 网络连接正常"
else
    echo "⚠️  网络连接可能有问题"
fi

echo ""
echo "🎯 检查完成！"

# 给出建议
echo ""
echo "💡 建议:"
if [[ "$CONDA_DEFAULT_ENV" != "env_isaaclab" ]]; then
    echo "1. 激活 conda 环境: conda activate env_isaaclab"
fi

if [[ -z "$ROS_DISTRO" ]]; then
    echo "2. 设置 ROS 2 环境: source /opt/ros/humble/setup.bash"
fi

if [[ ! -d "build" ]]; then
    echo "3. 构建工作空间: colcon build --symlink-install"
fi

echo "4. 如有问题，请运行: ./setup_env.sh"
