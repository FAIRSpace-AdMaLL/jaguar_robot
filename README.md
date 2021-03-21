# Jaguar_Sheffield

This ROS sample code is for Jaguar4x4 robot 2014 mode from Dr Robot Inc.
By default, the robot IP is "192.168.0.60", the main port number is "10001". The motor drive board is working in open loop or close loop velocity control mode.

After successfull connection, the program will publish IMU, GPS, motor sensor(encoder, temperature, ...), motor driver baord info.
You could use following cmd to check these messages:

rostopic echo /drrobot_imu
rostopic echo /drrobot_gps
rostopic echo /drrobot_motor
rostopic echo /drrobot_motorboard

This program will also subscribe to receive motor driving command.
Below command will release motor drive board EStop:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !MG'

Below command will drive robot forward:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !M 200 -200'

Below command will stop robot:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !M 0 0'


Below command will drive robot backward:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !M -200 200'

or you could use keyboard to control the robot:

rosrun jaguar4x4_2014 drrobot_keyboard_teleop_node

This program will use "WASD" key to move the robot, and release the key will stop the robot.
Before driving, you need press "z" key to release the robot from "EStop" state.