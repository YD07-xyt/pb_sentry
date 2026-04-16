DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/humble/setup.bash
source $DIR/install/setup.bash 
rviz2 -d /home/ma/code/nav_rm_2026/pb_sentry/src/pb_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz
