#!bin/bash

run="./run.sh"

serial="./serial.sh"

bash -c "run" &
pid_run=$!
echo "✅ 导航已启动,PID: $pid_run"

bash -c "serial"
pid_serial=$!
echo "✅ 串口已启动,PID: $pid_serial"

