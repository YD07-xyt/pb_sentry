source install/setup.bash  

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=mas \
slam:=False \
use_composition:=True \
use_robot_state_pub:=True