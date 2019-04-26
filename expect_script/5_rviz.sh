#!/usr/bin/expect

spawn ssh alvin@192.168.1.234
expect "password:"
send "ros\r"
set prompt_re {\$ $}
expect -re $prompt_re
send "cd ~/catkin_ws\r"
expect -re $prompt_re
send "rosrun rviz rviz -d catkin_ws/src/aruco_gridboard/data/aruco_grid.rviz\n"
interact
