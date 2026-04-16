#!/bin/bash

# 设置错误时退出
#set -e

# 日志文件路径
LOG_FILE="/home/ma/code/nav_rm_2026/pb_sentry/log_md/rmul.log"
exec 1> >(tee -a "$LOG_FILE")
exec 2>&1

echo "========================================"
echo "Starting robot services at $(date)"
echo "========================================"

# 设置ROS2环境变量（请根据实际路径修改）
export DIR="/home/ma/code/nav_rm_2026/pb_sentry"  # 修改为实际路径
export ROS_DOMAIN_ID=42  # 如果需要，设置ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # 如果需要指定RMW实现

# 等待网络和系统就绪（可选）
sleep 2



# cleanup() {
#     echo "Received shutdown signal, cleaning up..."
#     # 杀死所有子进程
#     pkill -P $$
#     exit 0
# }

# # 捕获 SIGTERM 和 SIGINT
# trap cleanup SIGTERM SIGINT

# 检查ROS2环境
if [ ! -f "$DIR/install/setup.bash" ]; then
    echo "Error: ROS2 setup.bash not found at $DIR/install/setup.bash"
    exit 1
fi

# Source ROS2环境
source "$DIR/install/setup.bash"
echo "Sourced ROS2 environment from $DIR/install/setup.bash"

# 启动标准机器人PP
echo "Starting standard_robot_pp_ros2..."
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py &
PID_PP=$!
echo "standard_robot_pp_ros2 started with PID: $PID_PP"

# 启动静态变换发布器
echo "Starting static_transform_publisher..."
ros2 run tf2_ros static_transform_publisher \
    --x 1.5 --y -2.0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map \
    --child-frame-id odom \
    --ros-args -r __ns:=/red_standard_robot1 &
PID_TF=$!
echo "static_transform_publisher started with PID: $PID_TF"

# 启动导航系统
echo "Starting navigation system..."
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    world:=rmul2026 \
    slam:=False \
    use_rviz:=True \
    use_composition:=True \
    use_robot_state_pub:=True &
PID_NAV=$!
echo "Navigation system started with PID: $PID_NAV"

# 等待一下
sleep 1

# 启动哨兵行为节点
echo "Starting sentry behavior..."
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py &
PID_SENTRY=$!
echo "Sentry behavior started with PID: $PID_SENTRY"

echo "========================================"
echo "All services started successfully at $(date)"
echo "Process IDs:"
echo "  standard_robot_pp: $PID_PP"
echo "  static_tf: $PID_TF"
echo "  navigation: $PID_NAV"
echo "  sentry_behavior: $PID_SENTRY"
echo "========================================"

# 等待所有后台进程（保持脚本运行）
wait $PID_PP $PID_TF $PID_NAV $PID_SENTRY
