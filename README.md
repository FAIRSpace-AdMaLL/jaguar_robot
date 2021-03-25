# Jaguar_Sheffield

This ROS package is for Jaguar4x4 robot 2014 mode from Dr Robot Inc. The robot manual can be found [here](http://jaguar.drrobot.com/images/Jaguar_4x4_wheel_manual.pdf). This package also includes launch files to run our sensors.

## Dependencies

*Only install the ones required for your sensor*

- Ouster OS1 [driver](https://github.com/ouster-lidar/ouster_example)
- zed2 [driver](https://github.com/stereolabs/zed-ros-wrapper)
- Xsense IMU driver

## Network setupOrganized

### Connecting to the robot

- SSID: `DriJaguar`
- Password: `drrobotdrrobot`
- Assign this static IP to your device: `192.168.0.104`
- Netmask: `255.255.255.0`

To test your connection: `ping 192.168.0.60`.

## Connecting to the Ouster-OS1 Lidar

`connect_lidar` is used to setup the network to communicate with lidar and setup ptp.

### One time setup

- Install ptp4l.

    ```
    sudo apt install -y linuxptp
    ```
- Make script excutable.
    ```
    cd ~/catkin_ws/src/jaguar_robot/scripts
    chmod +x connect_lidar
    ```
### Establishing a connection

- Power up the lidar.
- Run the script.

    ```
    cd ~/catkin_ws/src/jaguar_robot/scripts
    ./connect_lidar
    ```
- After the lidar has been on for ~20s, plug it in your ethernet port.

## Usage

- Run robot driver + joystick teleop + sensors (Ouster OS1, IMU, zed2 camera & fisheye camera)

    ```
    roslaunch jaguar4x4_2014 move_and_sense.launch
    ```

- Run robot driver + joystick only

    ```
    roslaunch jaguar4x4_2014 run_simple.launch
    ```

- or you could use keyboard to control the robot:

    ```
    rosrun jaguar4x4_2014 drrobot_keyboard_teleop_node
    ```

    This program will use "WASD" key to move the robot, and release the key will stop the robot.
    Before driving, you need press "z" key to release the robot from "EStop" state.
