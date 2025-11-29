MA-sentry-2026

## build
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## run
### 导航
```bash
source install/setup.bash  
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=rmuc_2025 \
slam:=False \
use_robot_state_pub:=True
```
### 建图
```bash
source install/setup.bash  
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

## 看tf
```bash
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1
```
```bash
ros2 run tf2_tools view_frames
```

### mid360
```bash
    source install/setup.bash
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### point_lio
```bash
source install/setup.bash
ROS2 launch point_lio point_lio.launch.py
```

## 建图补tf
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id odom \
  --child-frame-id base_footprint \
  --ros-args -r __ns:=/red_standard_robot1
```

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom --ros-args -r __ns:=/red_standard_robot1
```













