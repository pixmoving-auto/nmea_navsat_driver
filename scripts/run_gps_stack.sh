# 作者： Teddy Liu
# 日期： 2025-11-25
# 版本： 1.0.0
# 说明： 该脚本用于启动 gps 栈,作为FAE测试使用，能过快速启动ADCU的GPS功能，包括 gpsd 服务、nmea_gpsd_driver 和 rtk_config_web
# 使用方法：
# 1. 将脚本复制到 /home/nvidia/pix/robobus/autoware-robobus.master 目录下
# 2. 赋予脚本执行权限
# 3. 执行脚本
# 4. 输入 y 确认执行
# 5. 输入 n 跳过执行
# 6. 输入 q 退出脚本
# 7. 输入 h 查看帮助
# 8. 输入 v 查看版本
# 9. 输入 q 退出脚本

#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

# 用户确认函数
confirm_step() {
    local step_description="$1"
    local step_number="$2"
    local total_steps="$3"
    
    echo ""
    echo "=========================================="
    echo "[${step_number}/${total_steps}] ${step_description}"
    echo "=========================================="
    echo -n "是否执行此步骤? [y/N]: "
    read -r response
    
    case "$response" in
        [yY]|[yY][eE][sS])
            return 0
            ;;
        *)
            echo "跳过此步骤。"
            return 1
            ;;
    esac
}

# 步骤总数
TOTAL_STEPS=5

# [1/6] 检查 gpsd 服务状态
if confirm_step "检查 gpsd 服务状态" 1 $TOTAL_STEPS; then
    if systemctl is-active --quiet gpsd; then
        echo "gpsd 已运行，跳过重启。"
    else
        echo "gpsd 未运行，执行重启..."
        sudo systemctl restart gpsd
        echo "gpsd 服务已重启。"
    fi
fi

# [2/6] 配置 /dev/ttyTHS0 波特率
if confirm_step "配置 /dev/ttyTHS0 波特率为 460800" 2 $TOTAL_STEPS; then
    sudo stty -F /dev/ttyTHS0 460800
    echo "/dev/ttyTHS0 波特率已配置为 460800。"
fi

# [3/6] 配置 /dev/ttyTHS1 波特率
if confirm_step "配置 /dev/ttyTHS1 波特率为 460800" 3 $TOTAL_STEPS; then
    sudo stty -F /dev/ttyTHS1 speed 460800
    echo "/dev/ttyTHS1 波特率已配置为 460800。"
fi

# [4/6] 载入 ROS 2 环境
if confirm_step "载入 ROS 2 环境" 4 $TOTAL_STEPS; then
    set +u
    source "${SCRIPT_DIR}/install/setup.bash"
    set -u
    echo "ROS 2 环境已载入。"
fi

# [5/5～6] 启动 nmea_gpsd_driver 和 rtk_config_web
if confirm_step "启动 nmea_gpsd_driver 和 rtk_config_web" 5 $TOTAL_STEPS; then
    set +u
    if [ -z "${ROS_DISTRO:-}" ]; then
        echo "警告: ROS 2 环境未载入，正在尝试载入..."
        source "${SCRIPT_DIR}/install/setup.bash"
    fi
    set -u
    
    echo "启动 nmea_gpsd_driver (后台)..."
    ros2 launch nmea_navsat_driver nmea_gpsd_driver.launch.py &
    gpsd_pid=$!
    echo "nmea_gpsd_driver 已启动 (PID: $gpsd_pid)"
    
    echo "启动 rtk_config_web (前台)..."
    ros2 launch rtk_config_web_v2 rtk_config_web.launch.py
    
    # 等待后台进程
    wait $gpsd_pid
fi

echo ""
echo "=========================================="
echo "所有步骤执行完成！"
echo "=========================================="

