DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash  
ros2 run nav2_map_server map_saver_cli -f /home/ma/code/nav_rm_2026/pb_sentry/src/pb_sentry_nav/pb2025_nav_bringup/map/reality/test_map

cp  /home/ma/code/nav_rm_2026/pb_sentry/src/pb_sentry_nav/p `
suoint_lio/PCD/scans.pcd /home/ma/code/nav_rm_2026/pb_sentry/src/pb_sentry_nav/pb2025_nav_bringup/pcd/reality/test_map.pcd