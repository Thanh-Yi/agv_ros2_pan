#!/bin/bash

echo "Set permission for /dev/ttyUSB0"
sudo chmod 666 /dev/ttyUSB0

echo "Start encoder node"
python3 ~/ros2_ws/src/mobile/read_velocity/encoder.py &

sleep 1

echo "Start send_velocity node"
python3 ~/ros2_ws/src/mobile/read_velocity/send_velocity.py &

sleep1

echo "Start imu" & 
python3 ~/ros2_ws/src/mobile/scripts/hwt901b_imu.py &

sleep1 

echo "Start lidar" & 
python3 ~/ros2_ws/src/mobile/scripts/lds50cr.py 