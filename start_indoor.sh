#!/bin/bash
source /home/robot/.bashrc
source /home/robot/.profile
source /opt/ros/indigo/setup.bash
nohup rostopic pub -1 /start_indoor std_msgs/Int32 1 >/home/robot/Desktop/syw/shell/log.txt 2>/home/robot/Desktop/syw/shell/log.txt &
