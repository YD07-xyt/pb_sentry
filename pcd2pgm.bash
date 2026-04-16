DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/install/setup.bash  
ros2 launch pcd2pgm pcd2pgm_launch.py
