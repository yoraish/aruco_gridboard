# Indoor autonomous flight with Arducopter, ROS and Aruco Boards Detection

[video]

I would like to show my work an a system capable to realize indoor autonomous flight.
The system is based on a quadcopter with a Raspberry Pi 3 and a raspicam 2. Images from camera are used to calculate pose estimation on the Raspberry Pi, the result are sent as mavlink messasges to the Flight Controller.
The camera is downward looking and on the floor there is an Aruco Boards like this:

[img]

The pose estimation is calculated by aruco_gridboard ROS package on the Raspberry Pi and the relevant messages are sent to the Flight Controller using mavros ROS package.

The Flight Controller and the Raspberry Pi 3 on the quadcopter are connected via serial port whereas the Rapsberry Pi 3 and the desktop PC are connected via WiFi. The desktop PC is used only for configuration and visualization purpuses.

## Components of the system

- A little quadcopter (160mm) with Revolution FC with Arducopter 3.7-dev and the following relevant parameters:
  - AHRS_EKF_TYPE 2
  - EKF2_ENABLE 1
  - EKF3_ENABLE 0
  - EK2_GPS_TYPE 3
  - EK2_POSNE_M_NSE 0.1
  - EK2_EXTNAV_DELAY 80
  - GPS_TYPE 0
  - COMPASS_USE 0
  - VISO_TYPE 0
	
- On the quadcopter there is a Raspberry Pi 3 (connected to FC with serial port) and a Raspberry Cam
- On the Raspberry Pi there is ROS Kinetic with mavros and aruco_gridboard packages
- The video is captured with raspicam_node witch publish camera/image and camera/camera_info topics
- On the Raspberry Pi aruco_gridboard (slightly modified by me) subscribe to above topics and publish a camera_pose message to the mavros/vision_pose/pose topic.

A SET_GPS_GLOBAL_ORIGIN and a SET_HOME_POSITION messages (https://github.com/anbello/aruco_gridboard/blob/master/script/set_origin.py) are sent before starting to use the system.

## Instructions to reproduce the system

On the Raspberry Pi 3 on quadcopter:
- Install Ubuntu 16.04 and ROS Kinetic with Ubiquity Robotics Raspberry Pi images (https://downloads.ubiquityrobotics.com/pi.html)
- Edit /boot/config.txt to have higher serial speed on /dev/ttyAMA0
```
find the row with #init_uart_clock=3000000 and change it in this way: init_uart_clock=16000000
at the end of the file comment all lines after # Allow UART and Bluetooth ...
add the line: dtoverlay=pi3-disable-bt
reboot
```
- Connect the serial port with one telemetry port on the FC

On the desktop PC:
- Install ROS Kinetic on Ubuntu 16.04 (http://wiki.ros.org/kinetic/Installation/Ubuntu), maybe newer version work the same but I did not tested
- Install ros-kinetic-joy-teleop (sudo apt ros-kinetic-joy-teleop) and configure for your gamepad
```
I use a gamepad instead of RC because using 2.4GHz RC disturb the WiFi video streaming
In mavros there is a configuration file for Logitech F710 gamepad
```
- Install mavros (sudo apt ros-kinetic-mavros*)
- If you are not familiar with ROS follow the tutorials (http://wiki.ros.org/ROS/Tutorials)
- Clone my fork of aruco_gridboard (https://github.com/anbello/aruco_gridboard) in ~/catkin_ws/src
- Build all
```
cd ~/catkin_ws
catkin_make
```

Now to start all the node needed by the system to work give the following command on different term (tab)
(in my system 192.168.10.16 is the PC and 192.168.10.10 is the Raspberry Pi on the quadcopter)

tab 1:
```
ssh ubuntu@ubiquityrobot
(login)
ubuntu@ubiquityrobot:~/catkin_ws$ roslaunch aruco_gridboard detection_rpicam.launch
```

tab2:
```
ssh ubuntu@ubiquityrobot
(login)
ubuntu@ubiquityrobot:~/catkin_ws$ roslaunch mavros apm.launch fcu_url:=/dev/ttyAMA0:921600 gcs_url:=tcp-l://192.168.10.10:2000
```

tab3:
```
ssh ubuntu@ubiquityrobot
(login)
ubuntu@ubiquityrobot:~/catkin_ws$ rosrun aruco_gridboard set_origin.py (only after receiving EK2 ...)
```

tab4:
```
ssh ubuntu@ubiquityrobot
(login)
ubuntu@ubiquityrobot:~/catkin_ws$ rosrun aruco_gridboard mavros_control.py 
```

tab5:
```
andrea@galileo:~/catkin_ws$ export ROS_MASTER_URI="http://ubiquityrobot.local:11311"
andrea@galileo:~/catkin_ws$ rosrun rqt_reconfigure rqt_reconfigure (for setting camera params then exit)
andrea@galileo:~/catkin_ws$ roslaunch mavros_extras teleop.launch
```

tab6:
```
andrea@galileo:~/catkin_ws$ export ROS_MASTER_URI="http://ubiquityrobot.local:11311"
andrea@galileo:~/catkin_ws$ rosrun rviz rviz
```

[img]

On PC you also have to run a GCS of your choice (connected in tcp to 192.168.10.10:2000) to configure, see telemetry data, mavlink inspector, give commands, ...
All of this things can be done also via ROS messages and services but in this way could be easier.
