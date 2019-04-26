#!/usr/bin/expect

spawn ssh alvin@192.168.1.234
expect "password:"
send "ros\r"
set prompt_re {\$ $}
expect -re $prompt_re
send "cd ~/catkin_ws\r"
expect -re $prompt_re
send "rosservice call /mavros/set_stream_rate 0 100 1\n"
expect -re $prompt_re
send "rosrun aruco_gridboard set_origin.py\n"
interact
