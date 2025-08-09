#!/bin/bash

# RL_AIHandkerchief 环境配置脚本
# 用于快速设置 ROS 2 + Isaac Lab 开发环境

set -e

echo "🚀 开始配置 RL_AIHandkerchief 环境..."

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查 conda 环境
check_conda_env() {
    log_info "检查 conda 环境..."
    
    if ! command -v conda &> /dev/null; then
        log_error "conda 未安装，请先安装 miniconda 或 anaconda"
        exit 1
    fi
    
    if [[ "$CONDA_DEFAULT_ENV" != "env_isaaclab" ]]; then
        log_warning "当前不在 env_isaaclab 环境中"
        log_info "尝试激活 env_isaaclab 环境..."
        
        # 检查环境是否存在
        if conda env list | grep -q "env_isaaclab"; then
            log_info "请运行: conda activate env_isaaclab"
            log_info "然后重新运行此脚本"
            exit 1
        else
            log_error "env_isaaclab 环境不存在，请先创建 Isaac Lab 环境"
            exit 1
        fi
    fi
    
    log_success "conda 环境检查完成: $CONDA_DEFAULT_ENV"
}

# 检查 ROS 2 环境
check_ros2_env() {
    log_info "检查 ROS 2 环境..."
    
    if [[ -z "$ROS_DISTRO" ]]; then
        log_warning "ROS 2 环境未设置，尝试自动配置..."
        if [[ -f "/opt/ros/humble/setup.bash" ]]; then
            source /opt/ros/humble/setup.bash
            log_success "已加载 ROS 2 Humble 环境"
        else
            log_error "未找到 ROS 2 Humble 安装"
            exit 1
        fi
    else
        log_success "ROS 2 环境已设置: $ROS_DISTRO"
    fi
}

# 安装必需的 Python 依赖
install_dependencies() {
    log_info "检查并安装必需的 Python 依赖..."
    
    # 在当前 conda 环境中安装
    log_info "在 conda 环境中安装依赖..."
    pip install catkin_pkg empy==3.3.4 lark --quiet
    
    # 检查系统 Python 是否需要安装
    SYSTEM_PYTHON="/home/gift/miniconda3/bin/python3"
    if [[ -f "$SYSTEM_PYTHON" ]]; then
        log_info "在系统 Python 中安装依赖..."
        $SYSTEM_PYTHON -m pip install catkin_pkg empy==3.3.4 lark --quiet
    fi
    
    log_success "Python 依赖安装完成"
}

# 验证依赖
verify_dependencies() {
    log_info "验证依赖安装..."
    
    # 验证当前环境
    python3 -c "import catkin_pkg; print('✓ catkin_pkg')" || { log_error "catkin_pkg 导入失败"; exit 1; }
    python3 -c "import em; print('✓ empy')" || { log_error "empy 导入失败"; exit 1; }
    python3 -c "import lark; print('✓ lark')" || { log_error "lark 导入失败"; exit 1; }
    
    # 验证系统 Python (如果存在)
    SYSTEM_PYTHON="/home/gift/miniconda3/bin/python3"
    if [[ -f "$SYSTEM_PYTHON" ]]; then
        log_info "验证系统 Python 依赖..."
        $SYSTEM_PYTHON -c "import catkin_pkg; print('✓ 系统 catkin_pkg')" 2>/dev/null || log_warning "系统 catkin_pkg 可能有问题"
        $SYSTEM_PYTHON -c "import em; print('✓ 系统 empy')" 2>/dev/null || log_warning "系统 empy 可能有问题"
        $SYSTEM_PYTHON -c "import lark; print('✓ 系统 lark')" 2>/dev/null || log_warning "系统 lark 可能有问题"
    fi
    
    log_success "依赖验证完成"
}

# 构建工作空间
build_workspace() {
    log_info "构建 ROS 2 工作空间..."
    
    # 确保在正确的目录
    if [[ ! -f "$(pwd)/src/piper_utils/piper/package.xml" ]]; then
        log_error "请在 RL_AIHandkerchief 根目录下运行此脚本"
        exit 1
    fi
    
    # 清理之前的构建
    if [[ -d "build" ]]; then
        log_info "清理之前的构建..."
        rm -rf build install log
    fi
    
    # 构建
    log_info "开始构建..."
    if colcon build --symlink-install; then
        log_success "工作空间构建成功"
    else
        log_error "工作空间构建失败"
        exit 1
    fi
}

# 设置工作空间环境
setup_workspace() {
    log_info "设置工作空间环境..."
    
    if [[ -f "install/setup.bash" ]]; then
        source install/setup.bash
        log_success "工作空间环境已设置"
    else
        log_error "未找到 install/setup.bash"
        exit 1
    fi
}

# 创建环境脚本
create_env_scripts() {
    log_info "创建环境脚本..."
    
    # 创建 setup_ros_env.sh
    cat > setup_ros_env.sh << 'EOF'
#!/bin/bash
# ROS 2 环境设置脚本

# 设置 ROS 2 环境
source /opt/ros/humble/setup.bash

# 设置工作空间环境（如果存在）
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
    echo "✓ ROS 2 工作空间环境已加载"
else
    echo "⚠ 工作空间未构建，请先运行 colcon build"
fi
EOF
    chmod +x setup_ros_env.sh
    
    # 创建 .vscode/settings.json (如果需要)
    if [[ ! -d ".vscode" ]]; then
        mkdir -p .vscode
        cat > .vscode/settings.json << 'EOF'
{
    "python.defaultInterpreterPath": "/home/gift/miniconda3/envs/env_isaaclab/bin/python",
    "ros.distro": "humble",
    "files.associations": {
        "*.launch": "xml",
        "*.xacro": "xml",
        "*.urdf": "xml"
    },
    "editor.rulers": [100],
    "editor.insertSpaces": true,
    "editor.tabSize": 4
}
EOF
    fi
    
    log_success "环境脚本创建完成"
}

# 主函数
main() {
    echo "================================================================"
    echo "    RL_AIHandkerchief 环境配置工具"
    echo "================================================================"
    
    check_conda_env
    check_ros2_env
    install_dependencies
    verify_dependencies
    build_workspace
    setup_workspace
    create_env_scripts
    
    echo "================================================================"
    log_success "环境配置完成！"
    echo ""
    echo "📝 下次使用时，请运行以下命令激活环境："
    echo "   conda activate env_isaaclab"
    echo "   source setup_ros_env.sh"
    echo ""
    echo "🚀 或者直接运行："
    echo "   ./setup_ros_env.sh"
    echo "================================================================"
}

# 运行主函数
main "$@"
