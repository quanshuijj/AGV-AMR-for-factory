#!/bin/bash
# 车辆运动学验证快速启动脚本

echo "车辆运动学验证系统快速启动"
echo "================================"

# 检查并设置虚拟CAN接口
setup_vcan() {
    echo "检查虚拟CAN接口..."
    
    # 检查vcan模块
    if ! lsmod | grep -q "vcan"; then
        echo "加载vcan模块..."
        sudo modprobe vcan
    fi
    
    # 检查vcan0接口
    if ! ip link show vcan0 >/dev/null 2>&1; then
        echo "创建vcan0接口..."
        sudo ip link add dev vcan0 type vcan
    fi
    
    # 启动接口
    if ! ip link show vcan0 | grep -q "UP"; then
        echo "启动vcan0接口..."
        sudo ip link set up vcan0
    fi
    
    echo "虚拟CAN接口设置完成"
    echo "当前CAN接口状态:"
    ip link show vcan0
}

# 安装Python依赖
install_deps() {
    echo "检查Python依赖..."
    
    # 检查是否有pip
    if ! command -v pip3 >/dev/null 2>&1; then
        echo "错误: 未找到pip3, 请先安装python3-pip"
        exit 1
    fi
    
    # 安装依赖包
    echo "安装所需的Python包..."
    pip3 install --user python-can rospy geometry_msgs std_msgs gazebo_msgs tf
    
    echo "Python依赖安装完成"
}

# 显示使用说明
show_usage() {
    echo ""
    echo "使用说明:"
    echo "=========="
    echo "1. 在一个终端运行验证器 (接收和分析CAN数据):"
    echo "   python3 vehicle_kinematics_validator.py"
    echo ""
    echo "2. 在另一个终端运行测试发送器 (发送测试CAN数据):"
    echo "   python3 can_test_sender.py --mode test        # 运行完整测试套件"
    echo "   python3 can_test_sender.py --mode interactive # 交互模式"
    echo "   python3 can_test_sender.py --mode continuous  # 连续发送模式"
    echo ""
    echo "3. 或者运行你的C++程序，它会自动发送CAN数据到vcan0"
    echo ""
    echo "验证器将自动分析接收到的CAN消息并验证运动学模型的正确性"
    echo ""
}

# 检查CAN工具
check_can_tools() {
    echo "检查CAN工具..."
    
    if command -v candump >/dev/null 2>&1; then
        echo "CAN工具已安装"
        echo "可以使用以下命令监控CAN消息:"
        echo "  candump vcan0"
    else
        echo "建议安装can-utils以便调试:"
        echo "  sudo apt-get install can-utils"
    fi
}

# 主函数
main() {
    # 解析命令行参数
    SETUP_ONLY=false
    INSTALL_DEPS=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --setup-only)
                SETUP_ONLY=true
                shift
                ;;
            --install-deps)
                INSTALL_DEPS=true
                shift
                ;;
            --help|-h)
                echo "用法: $0 [选项]"
                echo "选项:"
                echo "  --setup-only    仅设置虚拟CAN接口"
                echo "  --install-deps  安装Python依赖"
                echo "  --help, -h      显示此帮助信息"
                exit 0
                ;;
            *)
                echo "未知选项: $1"
                exit 1
                ;;
        esac
    done
    
    # 执行相应操作
    if [ "$INSTALL_DEPS" = true ]; then
        install_deps
        exit 0
    fi
    
    setup_vcan
    check_can_tools
    
    if [ "$SETUP_ONLY" = true ]; then
        echo "虚拟CAN接口设置完成"
        exit 0
    fi
    
    show_usage
    
    # 询问是否立即启动
    echo ""
    read -p "是否立即启动验证器? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "启动验证器..."
        if [ -f "vehicle_kinematics_validator.py" ]; then
            python3 vehicle_kinematics_validator.py
        else
            echo "错误: 找不到 vehicle_kinematics_validator.py"
            echo "请确保脚本文件在当前目录中"
        fi
    fi
}

# 捕获中断信号
trap 'echo ""; echo "脚本被中断"; exit 1' INT

# 检查是否为root用户运行虚拟CAN设置
if [ "$EUID" -eq 0 ]; then
    echo "警告: 正在以root用户运行"
    echo "建议使用普通用户运行，脚本会在需要时请求sudo权限"
fi

# 运行主函数
main "$@"
