
```bash
[pointcloud_to_laserscan_node-7] [INFO] [1767744658.861051420] [pointcloud_to_laserscan]: Message Filter dropping message: frame 'odom' at time 1767744657.240 for reason 'discarding message because the queue is full'
```

[component_container_isolated-4] [INFO] [1768574036.484634021] [local_costmap.local_costmap]: Timed out waiting for transform from gimbal_yaw_fake to odom to become available, tf error: Could not find a connection between 'odom' and 'gimbal_yaw_fake' because they are not part of the same tree.Tf has two or more unconnected trees.
[component_container_isolated-4] warning: target point cloud is too small. |target|=0
[component_container_isolated-4] Magick: abort due to signal 11 (SIGSEGV) "Segmentation Fault"...
[ERROR] [component_container_isolated-4]: process has died [pid 9735, exit code -6, cmd '/opt/ros/humble/lib/rclcpp_components/component_container_isolated --ros-args --log-level info --ros-args -r __node:=nav2_container -r __ns:=/ --params-file /tmp/launch_params_jnismzrh --params-file /tmp/launch_params_gfqwue5n -r /tf:=tf -r /tf_static:=tf_static'].