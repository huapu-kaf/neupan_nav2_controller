#!/bin/bash
# NeuPAN Nav2 控制器插件验证脚本
# 完整的安装、构建和测试流程验证
#
# 作者: NeuPAN Team  
# 许可证: GNU General Public License v3.0

set -e  # 遇到错误时立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 输出函数
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_step() {
    echo -e "\n${CYAN}🔍 $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${PURPLE}💡 $1${NC}"
}

# 全局变量
PACKAGE_NAME="neupan_nav2_controller"
WORKSPACE_DIR="${HOME}/neupan_nav2_ws"
INSTALL_SUCCESS=true
BUILD_SUCCESS=true
TEST_SUCCESS=true

# 检查系统要求
check_system_requirements() {
    print_header "检查系统要求"
    
    # 检查 Ubuntu 版本
    print_step "检查 Ubuntu 版本"
    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        if [[ "$VERSION_ID" == "22.04" ]]; then
            print_success "Ubuntu 22.04 LTS detected"
        else
            print_warning "推荐使用 Ubuntu 22.04 LTS，当前版本: $VERSION_ID"
        fi
    fi
    
    # 检查 ROS2
    print_step "检查 ROS2 安装"
    if command -v ros2 &> /dev/null; then
        ROS_VERSION=$(ros2 --version 2>/dev/null | head -n1)
        print_success "ROS2 已安装: $ROS_VERSION"
        
        if [[ -n "$ROS_DISTRO" ]]; then
            print_success "ROS_DISTRO: $ROS_DISTRO"
            if [[ "$ROS_DISTRO" != "humble" ]]; then
                print_warning "推荐使用 ROS2 Humble"
            fi
        else
            print_error "ROS2 环境未正确source"
            return 1
        fi
    else
        print_error "ROS2 未安装"
        return 1
    fi
    
    # 检查 Python
    print_step "检查 Python 版本"
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version)
        print_success "$PYTHON_VERSION"
    else
        print_error "Python3 未安装"
        return 1
    fi
    
    # 检查必要的工具
    print_step "检查构建工具"
    local tools=("colcon" "rosdep" "cmake" "make" "gcc" "g++")
    for tool in "${tools[@]}"; do
        if command -v $tool &> /dev/null; then
            print_success "$tool 已安装"
        else
            print_warning "$tool 未安装"
            INSTALL_SUCCESS=false
        fi
    done
}

# 检查依赖包
check_dependencies() {
    print_header "检查 ROS2 依赖包"
    
    local required_packages=(
        "nav2_core"
        "nav2_common"
        "nav2_bringup"
        "pluginlib"
        "geometry_msgs"
        "nav_msgs"
        "sensor_msgs"
        "tf2"
        "tf2_ros"
    )
    
    for package in "${required_packages[@]}"; do
        print_step "检查 $package"
        if ros2 pkg prefix $package &> /dev/null; then
            print_success "$package 已安装"
        else
            print_error "$package 未安装"
            INSTALL_SUCCESS=false
        fi
    done
    
    # 检查 Python 依赖
    print_step "检查 Python 依赖"
    local python_packages=("numpy" "torch")
    for package in "${python_packages[@]}"; do
        if python3 -c "import $package" &> /dev/null; then
            print_success "Python $package 已安装"
        else
            print_warning "Python $package 未安装"
        fi
    done
}

# 创建工作空间
setup_workspace() {
    print_header "设置工作空间"
    
    print_step "创建工作空间目录"
    if [[ ! -d "$WORKSPACE_DIR" ]]; then
        mkdir -p "$WORKSPACE_DIR/src"
        print_success "工作空间创建: $WORKSPACE_DIR"
    else
        print_info "工作空间已存在: $WORKSPACE_DIR"
    fi
    
    cd "$WORKSPACE_DIR"
    
    # 检查源码
    print_step "检查源码"
    if [[ -d "src/$PACKAGE_NAME" ]]; then
        print_success "源码已存在"
    else
        print_warning "源码不存在，请确保已正确复制项目文件"
        INSTALL_SUCCESS=false
    fi
}

# 安装依赖
install_dependencies() {
    print_header "安装依赖"
    
    if [[ "$INSTALL_SUCCESS" == "false" ]]; then
        print_step "安装缺失的依赖"
        
        # 更新包列表
        print_step "更新包列表"
        sudo apt update
        
        # 安装 ROS2 Nav2
        print_step "安装 Nav2"
        sudo apt install -y ros-$ROS_DISTRO-nav2-bringup \
                           ros-$ROS_DISTRO-nav2-core \
                           ros-$ROS_DISTRO-nav2-common \
                           ros-$ROS_DISTRO-navigation2
        
        # 安装开发工具
        print_step "安装开发工具"
        sudo apt install -y python3-colcon-common-extensions \
                           python3-rosdep \
                           build-essential \
                           cmake
                           
        # 使用 rosdep 安装依赖
        print_step "使用 rosdep 安装依赖"
        if [[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
            rosdep update
        else
            sudo rosdep init
            rosdep update
        fi
        
        rosdep install --from-paths src --ignore-src -r -y
        
        print_success "依赖安装完成"
    else
        print_info "所有依赖已满足"
    fi
}

# 构建项目
build_project() {
    print_header "构建项目"
    
    cd "$WORKSPACE_DIR"
    
    print_step "清理之前的构建"
    if [[ -d "build" ]] || [[ -d "install" ]]; then
        rm -rf build install log
        print_success "清理完成"
    fi
    
    print_step "构建 $PACKAGE_NAME"
    if colcon build --packages-select $PACKAGE_NAME --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        print_success "构建成功"
    else
        print_error "构建失败"
        BUILD_SUCCESS=false
        return 1
    fi
    
    print_step "Source 工作空间"
    source "$WORKSPACE_DIR/install/setup.bash"
    print_success "工作空间已source"
}

# 验证安装
verify_installation() {
    print_header "验证安装"
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    print_step "检查包是否可发现"
    if ros2 pkg prefix $PACKAGE_NAME &> /dev/null; then
        PACKAGE_PATH=$(ros2 pkg prefix $PACKAGE_NAME)
        print_success "包路径: $PACKAGE_PATH"
    else
        print_error "包未找到"
        TEST_SUCCESS=false
        return 1
    fi
    
    print_step "检查共享库"
    LIB_PATH="$PACKAGE_PATH/lib/lib$PACKAGE_NAME.so"
    if [[ -f "$LIB_PATH" ]]; then
        print_success "共享库找到: $LIB_PATH"
        print_info "文件大小: $(stat -c%s "$LIB_PATH") bytes"
    else
        print_error "共享库未找到: $LIB_PATH"
        TEST_SUCCESS=false
    fi
    
    print_step "检查插件XML"
    XML_PATH="$PACKAGE_PATH/share/$PACKAGE_NAME/neupan_controller_plugin.xml"
    if [[ -f "$XML_PATH" ]]; then
        print_success "插件XML找到: $XML_PATH"
    else
        print_error "插件XML未找到: $XML_PATH"
        TEST_SUCCESS=false
    fi
    
    print_step "检查配置文件"
    CONFIG_PATH="$PACKAGE_PATH/share/$PACKAGE_NAME/config/nav2_params.yaml"
    if [[ -f "$CONFIG_PATH" ]]; then
        print_success "配置文件找到: $CONFIG_PATH"
    else
        print_warning "配置文件未找到: $CONFIG_PATH"
    fi
}

# 运行测试
run_tests() {
    print_header "运行测试"
    
    cd "$WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
    
    # 运行插件注册测试
    print_step "运行插件注册测试"
    SCRIPT_PATH="$WORKSPACE_DIR/install/$PACKAGE_NAME/share/$PACKAGE_NAME/scripts/test_plugin_registration.py"
    if [[ -f "$SCRIPT_PATH" ]]; then
        if python3 "$SCRIPT_PATH"; then
            print_success "插件注册测试通过"
        else
            print_error "插件注册测试失败"
            TEST_SUCCESS=false
        fi
    else
        print_warning "插件注册测试脚本未找到"
    fi
    
    # 检查插件是否被 pluginlib 识别
    print_step "检查插件注册状态"
    if ros2 plugin list --packages $PACKAGE_NAME 2>/dev/null | grep -q "neupan_nav2_controller::NeuPANController"; then
        print_success "插件已正确注册到 pluginlib"
    else
        print_warning "插件可能未正确注册"
    fi
}

# 生成使用示例
generate_usage_examples() {
    print_header "生成使用示例"
    
    EXAMPLE_DIR="$WORKSPACE_DIR/neupan_examples"
    mkdir -p "$EXAMPLE_DIR"
    
    # 生成启动脚本
    cat > "$EXAMPLE_DIR/launch_neupan_nav2.sh" << 'EOF'
#!/bin/bash
# NeuPAN Nav2 启动示例脚本

# Source 工作空间
source ~/neupan_nav2_ws/install/setup.bash

# 启动 Nav2 和 NeuPAN 控制器
ros2 launch neupan_nav2_controller neupan_nav2_test.launch.py
EOF

    chmod +x "$EXAMPLE_DIR/launch_neupan_nav2.sh"
    
    # 生成测试脚本
    cat > "$EXAMPLE_DIR/test_neupan.sh" << 'EOF'
#!/bin/bash
# NeuPAN 控制器测试脚本

# Source 工作空间
source ~/neupan_nav2_ws/install/setup.bash

echo "运行 NeuPAN 控制器功能测试..."
python3 $(ros2 pkg prefix neupan_nav2_controller)/share/neupan_nav2_controller/scripts/test_neupan_plugin.py
EOF

    chmod +x "$EXAMPLE_DIR/test_neupan.sh"
    
    print_success "使用示例已生成到: $EXAMPLE_DIR"
    print_info "启动命令: $EXAMPLE_DIR/launch_neupan_nav2.sh"
    print_info "测试命令: $EXAMPLE_DIR/test_neupan.sh"
}

# 打印最终报告
print_final_report() {
    print_header "验证报告"
    
    echo -e "\n${CYAN}📊 验证结果总结${NC}"
    echo "=============================="
    
    if [[ "$INSTALL_SUCCESS" == "true" ]]; then
        echo -e "✅ 依赖检查: ${GREEN}通过${NC}"
    else
        echo -e "❌ 依赖检查: ${RED}失败${NC}"
    fi
    
    if [[ "$BUILD_SUCCESS" == "true" ]]; then
        echo -e "✅ 项目构建: ${GREEN}成功${NC}"
    else
        echo -e "❌ 项目构建: ${RED}失败${NC}"
    fi
    
    if [[ "$TEST_SUCCESS" == "true" ]]; then
        echo -e "✅ 功能验证: ${GREEN}通过${NC}"
    else
        echo -e "❌ 功能验证: ${RED}失败${NC}"
    fi
    
    echo -e "\n${CYAN}💡 下一步操作建议${NC}"
    echo "========================"
    
    if [[ "$BUILD_SUCCESS" == "true" && "$TEST_SUCCESS" == "true" ]]; then
        echo -e "🎉 ${GREEN}恭喜！NeuPAN Nav2 控制器插件验证通过！${NC}"
        echo ""
        echo "您现在可以："
        echo "1. 使用示例启动脚本测试控制器"
        echo "2. 在您的机器人项目中集成 NeuPAN 控制器"
        echo "3. 根据需要调整配置参数"
        echo ""
        echo "启动命令:"
        echo "  source $WORKSPACE_DIR/install/setup.bash"
        echo "  ros2 launch neupan_nav2_controller neupan_nav2_test.launch.py"
        
    else
        echo -e "⚠️  ${YELLOW}验证过程中发现问题，请检查：${NC}"
        echo ""
        
        if [[ "$INSTALL_SUCCESS" == "false" ]]; then
            echo "🔧 依赖问题:"
            echo "   - 安装缺失的 ROS2 Nav2 包"
            echo "   - 确保所有系统依赖已安装"
        fi
        
        if [[ "$BUILD_SUCCESS" == "false" ]]; then
            echo "🔧 构建问题:"
            echo "   - 检查编译错误信息"
            echo "   - 确保所有头文件和库文件可访问"
            echo "   - 检查 CMakeLists.txt 配置"
        fi
        
        if [[ "$TEST_SUCCESS" == "false" ]]; then
            echo "🔧 功能问题:"
            echo "   - 检查插件是否正确注册"
            echo "   - 验证配置文件是否正确安装"
            echo "   - 确保 Python 依赖已安装"
        fi
    fi
    
    echo -e "\n${PURPLE}📚 更多帮助:${NC}"
    echo "- 查看项目 README.md"
    echo "- 运行详细测试脚本"
    echo "- 检查 ROS2 和 Nav2 文档"
}

# 主函数
main() {
    print_header "NeuPAN Nav2 控制器插件完整验证"
    echo -e "${CYAN}开始验证 NeuPAN Nav2 控制器插件安装和配置${NC}"
    
    # 运行所有检查步骤
    check_system_requirements || true
    check_dependencies || true
    setup_workspace || true
    install_dependencies || true
    
    # 只有在前面步骤成功时才继续
    if [[ "$INSTALL_SUCCESS" == "true" ]]; then
        build_project || true
        
        if [[ "$BUILD_SUCCESS" == "true" ]]; then
            verify_installation || true
            run_tests || true
            generate_usage_examples || true
        fi
    fi
    
    print_final_report
    
    # 设置退出码
    if [[ "$BUILD_SUCCESS" == "true" && "$TEST_SUCCESS" == "true" ]]; then
        exit 0
    else
        exit 1
    fi
}

# 脚本入口
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi