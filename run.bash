#!/bin/bash

# 获取当前脚本所在目录的绝对路径，确保在任何地方执行都能找到子脚本
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

gnome-terminal --window --title="Serial_Node" -- bash -c "cd $DIR; ./serial.bash; exec bash" &
echo "✅ 串口窗口"

sleep 1

gnome-terminal --window --title="Navigation_Node" -- bash -c "cd $DIR; ./nav.bash; exec bash" &
echo "✅ 导航窗口"

sleep 4

gnome-terminal --window --title="bt_tree_Node" -- bash -c "cd $DIR; ./bt_tree.bash; exec bash" &
echo "✅ 决策行为树窗口"
