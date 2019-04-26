#!/usr/bin/expect

spawn ssh alvin@192.168.1.234
expect "password:"
send "ros\r"
set prompt_re {\$ $}
expect -re $prompt_re
send "cd ~/catkin_ws\r"
expect -re $prompt_re
send "cd src/aruco_gridboard/scriptr\r"
expect -re $prompt_re
send "ls\n"
expect -re $prompt_re
send "cd ~/catkin_ws\r"
expect -re $prompt_re
send "rosrun aruco_gridboard "
interact
