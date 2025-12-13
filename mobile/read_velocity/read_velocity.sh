#!/bin/bash

echo "Set permission for /dev/ttyUSB0"
sudo chmod 666 /dev/ttyUSB0

echo "Start encoder node"
python3 ~/ros2_ws/src/mobile/read_velocity/encoder.py &

sleep 1

echo "Start send_velocity node"
python3 ~/ros2_ws/src/mobile/read_velocity/send_velocity.py
