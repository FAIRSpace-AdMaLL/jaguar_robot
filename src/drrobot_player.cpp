/*!
 *  drrobot_player
 *  Copyright (c) 2014, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_player is a driver for motion control system on Jaguar series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.

$ drrobot_player
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "motor_cmd_sub"/std_msgs::string : motor commands to drive the robot.

Publishes to (name / type):
-@b drrobot_motor/MotorDataArray: will publish MotorDataArray Message. Please referee the message file.
-@b drrobot_motorboard/MotorDriverBoardArray: will publish MotorBoardArray Message. Please referee the message file.
-@b drrobot_gps/GPSInfo: will publish GPS Message
-@b drrobot_imu/IMUData: will publish IMU Message
<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: Jaguar
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <jaguar4x4_2014/MotorData.h>
#include <jaguar4x4_2014/MotorDataArray.h>
#include <jaguar4x4_2014/MotorBoardInfoArray.h>
#include <jaguar4x4_2014/MotorBoardInfo.h>
#include <jaguar4x4_2014/GPSInfo.h>
#include <jaguar4x4_2014/IMUData.h>

#include <DrRobotMotionSensorDriver.hpp>
#include "CTimer.h"

#define MOTOR_NUM           4       //max
#define MOTOR_BOARD_NUM     2       //max
using namespace std;
using namespace DrRobot_MotionSensorDriver;

class DrRobotPlayerNode
{
public:

    ros::NodeHandle node_;


    ros::Publisher motorInfo_pub_;
    ros::Publisher motorBoardInfo_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher odometryPub;
    ros::Subscriber motor_cmd_sub_;	
    std::string robot_prefix_;
    nav_msgs::Odometry odometry;

    DrRobotPlayerNode()
    {
        ros::NodeHandle private_nh("~");

        robotID_ = "DrRobot";
        private_nh.getParam("RobotID",robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        robotType_ = "Jaguar";
        private_nh.getParam("RobotType",robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        robotCommMethod_ = "Network";
        private_nh.getParam("RobotCommMethod",robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        robotIP_ = "192.168.0.60";
        private_nh.getParam("RobotBaseIP",robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        commPortNum_ = 10001;
        private_nh.getParam("RobotPortNum",commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("RobotSerialPort",robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        motorDir_ = 1;
        private_nh.getParam("MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.135; //0.135;
        private_nh.getParam("WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.35;//0.52; 
        private_nh.getParam("WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 1.0;
        private_nh.getParam("MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 380;
        private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
          robotConfig1_.commMethod = Network;
        }
        else
        {
          robotConfig1_.commMethod = Serial;
        }

        if (robotType_ == "Jaguar")
        {
          robotConfig1_.robotType = Jaguar;
        }
        robotConfig1_.portNum = commPortNum_;


	  strcpy(robotConfig1_.robotIP,robotIP_.c_str());


	  strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());

        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<jaguar4x4_2014::MotorDataArray>("drrobot_motor", 1);
        motorBoardInfo_pub_ = node_.advertise<jaguar4x4_2014::MotorBoardInfoArray>("drrobot_motorboard", 1);
        gps_pub_ = node_.advertise<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1);
        imu_pub_ = node_.advertise<jaguar4x4_2014::IMUData>("drrobot_imu", 1);
      	
	//Odometry publish
        odometryPub = node_.advertise<nav_msgs::Odometry>("/odom", 1);
	
	drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        if (  (robotType_ == "Jaguar") )
        {
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        }
        else
        {

        }
        cntNum_ = 0;
    }

    ~DrRobotPlayerNode()
    {
    }

    int start()
    {

      int res = -1;
      if (  (robotType_ == "Jaguar"))
      {
        res = drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
	if (res == 0)
	{
		ROS_INFO("open port number at: [%d]", robotConfig1_.portNum);
	}
	else
	{
		ROS_INFO("could not open network connection to [%s,%d]",  robotConfig1_.robotIP,robotConfig1_.portNum);
		//ROS_INFO("error code [%d]",  res);
	}

      }
      else
      {

      }


      //motor_cmd_sub_ = node_.subscribe<std_msgs::String>("drrobot_motor_cmd", 1, boost::bind(&DrRobotPlayerNode::cmdReceived, this, _1));
      motor_cmd_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdReceived, this, _1));
	
      //Reset odometry	     
      odomX=odomY=odomPhi=lastTime=lastForward=lastTurn=0;
      odoTimer.reset();                                                                                                              
      odoTimer.start(); 

      std::stringstream ss;
        
      ss << "MMW !MG";

      int nLen = strlen(ss.str().c_str());
      ROS_INFO("Active the controller");
      drrobotMotionDriver_->sendCommand(ss.str().c_str(), nLen);

      return(0);
    }

    int stop()
    {
        int status = 0;
        drrobotMotionDriver_->close();

        usleep(1000000);
        return(status);
    }

    void cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      	double g_vel = cmd_vel->linear.x;
        double t_vel = cmd_vel->angular.z * 1.5;
	      std::cout << " g vel" << g_vel << " t " << t_vel << std::endl;
        double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        
      	ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);

      	std::stringstream ss;
       
      	ss << "MMW !M " << leftWheelCmd; 
      	ss << " " << rightWheelCmd;
      	ss.seekg(0,std::ios::end);
      	int size = ss.tellg();
      	ss.seekg(0,std::ios::beg);
      	//cmdVel_string.data = ss.str();
      	//ss.str("");

        ROS_INFO("Received motor command: []");
      	std::cout << ss.str() << std::endl;
      	int nLen = strlen(ss.str().c_str());
        // ROS_INFO("Received motor command len: [%d]", nLen);
        drrobotMotionDriver_->sendCommand(ss.str().c_str(), nLen);
            
        // dead-reckoning (odometry) estimation 
      	float odometryTime = odoTimer.getTime()/1000.0;
      	std::cout << "Time " << odometryTime << std::endl;
      	float displacement = lastForward*(odometryTime-lastTime);
      	odomX += displacement*cos(odomPhi);
      	odomY += displacement*sin(odomPhi);
      	odomPhi = lastTurn; 
      	lastTime = odometryTime; 
      	lastForward = g_vel;
      	lastTurn = t_vel;
      	std::cout << "Odometry x " << odomX << " y "<< odomY << " phi " << odomPhi << std::endl;
        odometry.pose.pose.position.x = odomX;
      	odometry.pose.pose.position.y = odomY;
      	odometry.pose.pose.position.z = 0;
      	tf::Quaternion orientation;
      	orientation.setRPY(0,0,odomPhi);
      	odometry.pose.pose.orientation.x = orientation[0];
     	  odometry.pose.pose.orientation.y = orientation[1];
      	odometry.pose.pose.orientation.z = orientation[2];
        odometry.pose.pose.orientation.w = orientation[3];
	      odometry.header.frame_id = "odom"; 
      	odometryPub.publish(odometry);

        // ROS_INFO("publish GPS Info");
    }

    void doUpdate()
    {
	//int test = imuSensorData_.seq;
        drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
        drrobotMotionDriver_->readMotorBoardData(&motorBoardData_);
        drrobotMotionDriver_->readIMUSensorData(&imuSensorData_);
        drrobotMotionDriver_->readGPSSensorData(&gpsSensorData_);
	//test = test - imuSensorData_.seq;
	//ROS_INFO("IMU Packet : [%d]",test);
              // Translate from driver data to ROS data
            cntNum_++;
              jaguar4x4_2014::MotorDataArray motorDataArray;
              motorDataArray.motorData.resize(MOTOR_NUM);
              for (uint32_t i = 0 ; i < MOTOR_NUM; ++i)
              {
                  motorDataArray.motorData[i].header.stamp = ros::Time::now();
                  motorDataArray.motorData[i].header.frame_id = string("drrobot_motor_");
                  motorDataArray.motorData[i].header.frame_id += boost::lexical_cast<std::string>(i);

                  motorDataArray.motorData[i].motorPower = motorSensorData_.motorSensorPWM[i];
                  motorDataArray.motorData[i].encoderPos += motorSensorData_.motorSensorEncoderPos[i];
                  motorDataArray.motorData[i].encoderVel = motorSensorData_.motorSensorEncoderVel[i];
                  motorDataArray.motorData[i].encoderDiff = motorSensorData_.motorSensorEncoderPosDiff[i];
                  motorDataArray.motorData[i].motorTemp = motorSensorData_.motorSensorTemperature[i];
                  motorDataArray.motorData[i].motorCurrent = motorSensorData_.motorSensorCurrent[i];
              }

              //ROS_INFO("publish motor info array");
              motorInfo_pub_.publish(motorDataArray);

              jaguar4x4_2014::MotorBoardInfoArray motorBoardInfoArray;
              motorBoardInfoArray.motorBoardInfo.resize(MOTOR_BOARD_NUM);
              for (uint32_t i = 0 ; i < MOTOR_BOARD_NUM; ++i)
              {
                  motorBoardInfoArray.motorBoardInfo[i].header.stamp = ros::Time::now();
                  motorBoardInfoArray.motorBoardInfo[i].header.frame_id = string("drrobot_motor_");
                  motorBoardInfoArray.motorBoardInfo[i].header.frame_id += boost::lexical_cast<std::string>(i);

                  motorBoardInfoArray.motorBoardInfo[i].status = motorBoardData_.status[i];
                  motorBoardInfoArray.motorBoardInfo[i].temp1 = motorBoardData_.temp1[i];
                  motorBoardInfoArray.motorBoardInfo[i].temp2 = motorBoardData_.temp2[i];
                  motorBoardInfoArray.motorBoardInfo[i].temp3 = motorBoardData_.temp3[i];

                  motorBoardInfoArray.motorBoardInfo[i].volMain = motorBoardData_.volMain[i];
                  motorBoardInfoArray.motorBoardInfo[i].vol12V = motorBoardData_.vol12V[i];
                  motorBoardInfoArray.motorBoardInfo[i].vol5V = motorBoardData_.vol5V[i];
                  motorBoardInfoArray.motorBoardInfo[i].dinput = motorBoardData_.dinput[i];
                  motorBoardInfoArray.motorBoardInfo[i].doutput = motorBoardData_.doutput[i];
                  motorBoardInfoArray.motorBoardInfo[i].ack = motorBoardData_.ack[i];
              }

              //ROS_INFO("publish motor driver board info array");
              motorBoardInfo_pub_.publish(motorBoardInfoArray);

              jaguar4x4_2014::IMUData imuData;
              imuData.header.stamp = ros::Time::now();
              imuData.header.frame_id = string("drrobot_imu_");
              imuData.header.frame_id += boost::lexical_cast<std::string>(cntNum_);

              imuData.seq = imuSensorData_.seq;
              imuData.yaw = imuSensorData_.yaw;
              imuData.pitch = imuSensorData_.pitch;
              imuData.roll = imuSensorData_.roll;

              imuData.gyro_x = imuSensorData_.gyro_x;
              imuData.gyro_y = imuSensorData_.gyro_y;
              imuData.gyro_z = imuSensorData_.gyro_z;

              imuData.accel_x = imuSensorData_.accel_x;
              imuData.accel_y = imuSensorData_.accel_y;
              imuData.accel_z = imuSensorData_.accel_z;

              imuData.comp_x = imuSensorData_.comp_x;
              imuData.comp_y = imuSensorData_.comp_y;
              imuData.comp_z = imuSensorData_.comp_z;

	  //    ROS_INFO("SeqNum [%d]",  imuData.seq );
          //    ROS_INFO("publish IMU sensor data");
              imu_pub_.publish(imuData);

              jaguar4x4_2014::GPSInfo gpsInfo;
              gpsInfo.header.stamp = ros::Time::now();
              gpsInfo.header.frame_id = string("drrobot_gps_");
              gpsInfo.header.frame_id += boost::lexical_cast<std::string>(cntNum_);

	      gpsInfo.time = gpsSensorData_.timeStamp;
	      gpsInfo.date = gpsSensorData_.dateStamp;
              gpsInfo.status = gpsSensorData_.gpsStatus;
              gpsInfo.latitude = gpsSensorData_.latitude;
              gpsInfo.longitude = gpsSensorData_.longitude;
              gpsInfo.vog = gpsSensorData_.vog;
              gpsInfo.cog = gpsSensorData_.cog;
      	      
	      

              gps_pub_.publish(gpsInfo);
	       
		//send ping command here
              drrobotMotionDriver_->sendCommand("PING",4);

    }


private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;

    struct DrRobotMotionConfig robotConfig1_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct MotorBoardData motorBoardData_;
    struct IMUSensorData imuSensorData_;
    struct GPSSensorData gpsSensorData_;


    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;

    int cntNum_;

    //odometry status
    CTimer odoTimer;
    float lastTime,lastForward,lastTurn;
    float odomX,odomY,odomPhi;
};






int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaguar4x4_2014_node");

    DrRobotPlayerNode drrobotPlayer;
    ros::NodeHandle n;
    // Start up the robot
    if (drrobotPlayer.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

    
    ros::Rate loop_rate(50);      //10Hz


    while (n.ok())
    {
      drrobotPlayer.doUpdate();
      	
      
      ros::spinOnce();
     loop_rate.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    drrobotPlayer.stop();

    return(0);
}

