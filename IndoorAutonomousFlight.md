# Indoor autonomous flight with Arducopter, ROS and Aruco Boards Detection

<a href="http://www.youtube.com/watch?feature=player_embedded&v=dYhvNhqobVg" target="_blank"><img src="http://img.youtube.com/vi/dYhvNhqobVg/0.jpg" 
alt="" width="400" height="300" border="10" /></a>

This wiki page describes how to setup a system capable to realize indoor autonomous flight.
The system is based on a quadcopter with a Raspberry Pi 3 and a Raspberry Pi Camera Module v2. Images from camera are used to calculate poses estimation on the Raspberry Pi and the result are sent as mavlink messages to the Flight Controller.
The camera is downward looking and on the floor there is an Aruco Boards like this:

![alt text](https://discuss.ardupilot.org/uploads/default/original/3X/9/2/925b791aa83d644afaa14cf1c103b32a0875a72e.png "")

The system uses [ROS](http://www.ros.org/) for all the tasks it has to do. The images from Raspberry Pi Camera are captured by [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node), the poses estimation are calculated by a modified version of [aruco_gridboard](https://github.com/anbello/aruco_gridboard) and the relevant messages are sent to the Flight Controller using [mavros](http://wiki.ros.org/mavros). All this ROS packages, and other we will see later, runs on the Raspberry Pi 3.

The ROS node raspicam_node publish camera/image and camera/camera_info topics, the node aruco_gridboard subscribes to these topics and publish a camera_pose message to the mavros/vision_pose/pose topic, mavros translates ROS messages in mavlink messages and send it to the Flight Controller.

The messages SET_GPS_GLOBAL_ORIGIN and a SET_HOME_POSITION are sent with a [script](https://github.com/anbello/aruco_gridboard/blob/master/script/set_origin.py) before starting to use the system. 

The Flight Controller and the Raspberry Pi 3 on the quadcopter are connected via serial port whereas the Rapsberry Pi 3 and the desktop PC are connected via WiFi. The desktop PC is used only for configuration and visualization purposes. [rviz](http://wiki.ros.org/rviz) from ROS is used for visualization on PC.

## Components of the system

- A little quadcopter (160mm) with Revolution Flight Controller with Arducopter 3.7-dev and the following relevant parameters:
  - AHRS_EKF_TYPE 2
  - BRD_RTC_TYPES 1
  - EKF2_ENABLE 1
  - EKF3_ENABLE 0
  - EK2_GPS_TYPE 3
  - EK2_POSNE_M_NSE 0.1
  - EK2_VELD_M_NSE 0.1
  - EK2_VELNE_M_NSE 0.1
  - EK2_EXTNAV_DELAY 80
  - GPS_TYPE 0
  - COMPASS_USE 0
  - COMPASS_USE2 0
  - COMPASS_USE3 0
  - SERIAL1_BAUD 921   (the serial port used to connect to Raspberry Pi)
  - SERIAL1_PROTOCOL 2
  - SYSID_MYGCS 1   (to accept control from mavros)
  - VISO_TYPE 0
	
- On the quadcopter there is a Raspberry Pi 3 (connected to Flight Controller with serial port) and a Raspberry Pi Camera
- On the Raspberry Pi there is ROS Kinetic with raspicam_node, aruco_gridboard and mavros packages.

## Instructions to reproduce the system

### On the Raspberry Pi 3 on quadcopter
- Install Ubuntu 16.04 and ROS Kinetic with Ubiquity Robotics [Raspberry Pi images](https://downloads.ubiquityrobotics.com/pi.html)
- Edit /boot/config.txt to have higher serial speed on /dev/ttyAMA0 (connection at 921600 baud)
```
- find the row with #init_uart_clock=3000000 and change it in this way: init_uart_clock=16000000
- at the end of the file comment all lines after # Allow UART and Bluetooth ...
- add the line: dtoverlay=pi3-disable-bt
- reboot
```
- Connect the serial port with one telemetry port on the Flight Controller
- Connect to the PC using WiFi following the [instructions](https://learn.ubiquityrobotics.com/connect_network) on Ubiquity Robotics site
- Edit mavros configuration file apm_config.yaml to syncronize the flight controller and companion computer (Raspberry Pi) clocks using MAVLink’s SYSTEM_TIME and TIMESYNC messages as in this [wiki](http://ardupilot.org/dev/docs/ros-timesync.html)
- Calibrate the camera following the instructions in this [wiki](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
- Clone this fork of [aruco_gridboard](https://github.com/anbello/aruco_gridboard) in ~/catkin_ws/src
- Build all
```
cd ~/catkin_ws
catkin_make
```

### On the desktop PC
- Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) on Ubuntu 16.04 (maybe newer version work the same but was not tested)
- Install ros-kinetic-joy-teleop (sudo apt install ros-kinetic-joy-teleop) and configure for your Joystick
  - We use a Joystick instead of RC because using 2.4GHz RC disturb the WiFi video streaming. In mavros there is a configuration file for Logitech F710 Joystick, In the aruco_gridboard package we added a configuration file for the Xbox one Joystick.
- Install mavros (sudo apt install ros-kinetic-mavros*)
- If you are not familiar with ROS follow the [tutorials](http://wiki.ros.org/ROS/Tutorials)
- Edit ~/.bashrc and append the following line:
```
export ROS_MASTER_URI="http://ubiquityrobot.local:11311"
```
- Create a Catkin WorkSpace (on Raspberry Pi this is not necessary because it is already in Ubiquity Robotics image)
```
cd $HOME
mkdir -p catkin_ws/src
cd catkin_ws
catkin_init_workspace
```
- Clone this fork of [aruco_gridboard](https://github.com/anbello/aruco_gridboard) in ~/catkin_ws/src
- Build all
```
cd ~/catkin_ws
catkin_make
```

On PC you also have to run a GCS of your choice to configure the quadcopter, see telemetry data, see mavlink inspector, set flight modes and give commands. All of this things can be done also via ROS messages and services but in this way could be easier.

### Starting all ROS node
Now to start all the node needed by the system to work give the following command on different term (or tab with CTRL+SHIFT+T)
(in this example 192.168.10.16 is the PC and 192.168.10.10 is the Raspberry Pi on the quadcopter)

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
andrea@galileo:~/catkin_ws$ rosrun rqt_reconfigure rqt_reconfigure (for setting camera params then exit)
andrea@galileo:~/catkin_ws$ roslaunch mavros_extras teleop.launch
```

tab5:
```
andrea@galileo:~/catkin_ws$ rosrun rviz rviz -d catkin_ws/src/aruco_gridboard/data/aruco_grid.rviz
```

At this point it should be possible to see /mavros/vision_pose/pose and /mavros/local_position/pose, represented as 3 Axes, on rviz and moving the quadcopter with the camera towards the Aruco Board, you should see the two poses moving close to each other. Connecting the GCS to the quadcopter (tcp 192.168.10.10 2000) it should be possible to see the quadcopter on the map, set flight mode and give commands.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=zu6mK_AM5ks" target="_blank"><img src="http://img.youtube.com/vi/zu6mK_AM5ks/0.jpg" 
alt="" width="400" height="300" border="10" /></a>

If this last point is OK the first test could be done arming the quadcopter in Loiter mode, takeoff and hover over the Aruco Board with the Joystick, then land.

The last step (for now) is to test an all autonomous flight using one of the script included, to do this open another term or tab:
```
ssh ubuntu@ubiquityrobot
(login)
ubuntu@ubiquityrobot:~/catkin_ws$ rosrun aruco_gridboard mavros_control1.py 
```
You should see the quadcopter arm, takeoff, flight along the square and land as showed in the video at the beginning of this page.
