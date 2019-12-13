/*!
 * drrobot_keyboard_teleop.cpp
 * Copyright (c) 2014, Dr Robot Inc
 * All rights reserved.
 *
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
  drrobot_keyboard_teleop for demonstration and testing published std_msgs/String message to drrobot_player.
  It will use 4 keys to control robot move around
  a/A -- 0.5/1 turn to left
  w/W -- 0.5/1 forward
  d/D -- 0.5/1 turn to right
  s/S -- backward
  if no key pressed, it will stop robot
  x/X --  will EStop the Robot
  z/Z -- will relese the robot EStop
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
          then run drrobot_player first.
@verbatim
$ drrobot_keyboard_teleop
@endverbatim

<hr>
@section topic ROS topics

Publishes to (name / type):
-@b drrobot_motor_cmd: will publish drrobot_motor_cmd Message to drrobot_player.
<hr>
*/
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sstream>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_W_CAP 0x57
#define KEYCODE_A_CAP 0x41
#define KEYCODE_S_CAP 0x53
#define KEYCODE_D_CAP 0x44

#define KEYCODE_X 0x78
#define KEYCODE_Z 0x7A
#define KEYCODE_X_CAP 0x58
#define KEYCODE_Z_CAP 0x5A


class DrRobotKeyboardTeleopNode
{
    private:
        std_msgs::String cmdVel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        DrRobotKeyboardTeleopNode()
        {
            pub_ = n_.advertise<std_msgs::String>("drrobot_motor_cmd", 1);
            ros::NodeHandle n_private("~");
        }

        ~DrRobotKeyboardTeleopNode() { }
        void keyboardLoop();

        void stopRobot()
        {
  	   std::stringstream ss;
	   ss << "MMW !M 0 0";
	   cmdVel_.data = ss.str();
	  // ROS_INFO("Send control command [ %s]", cmdVel_.data.c_str());
            pub_.publish(cmdVel_);
        }
};

DrRobotKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"drrobot_teleope_keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    DrRobotKeyboardTeleopNode tbk;

    boost::thread t = boost::thread(boost::bind(&DrRobotKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void DrRobotKeyboardTeleopNode::keyboardLoop()
{
    char c;
    bool dirty = false;
    std::stringstream ss;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Press Z key to release EStop first");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");
    puts("Press X key to EStop robot");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_W:
		if (!dirty)
		{
			ss << "MMW !M 200 -200";	
			dirty = true;
		}
                break;
            case KEYCODE_S:
		if (!dirty)
		{

		        ss << "MMW !M -200 200";	
		        dirty = true;
		}
                break;

            case KEYCODE_A:
		if (!dirty)
		{
			ss << "MMW !M -200 -200";	
		        dirty = true;
		}
                break;
            case KEYCODE_D:
		if (!dirty)
		{
		        ss << "MMW !M 200 200";	
		        dirty = true;
		}
                break;

            case KEYCODE_W_CAP:
		if (!dirty)
		{
		        ss << "MMW !M 400 -400";	
		        dirty = true;
		}
                break;
            case KEYCODE_S_CAP:
		if (!dirty)
		{
		        ss << "MMW !M -400 400";
		        dirty = true;
		}
                break;
            case KEYCODE_A_CAP:
		if (!dirty)
		{
		        ss << "MMW !M -400 -400";
		        dirty = true;
		}
                break;
            case KEYCODE_D_CAP:
		if (!dirty)
		{
		        ss << "MMW !M 400 400";
		        dirty = true;
		}
                break;
            case KEYCODE_Z_CAP:
		if (!dirty)
		{
		        ss << "MMW !MG";
		        dirty = true;
		}
                break;
            case KEYCODE_Z:
		if (!dirty)
		{
		        ss << "MMW !MG";
		        dirty = true;
		}
                break;
            case KEYCODE_X:
		if (!dirty)
		{
		        ss << "MMW !EX";
		        dirty = true;
		}
                break;
            case KEYCODE_X_CAP:
		if (!dirty)
		{
		        ss << "MMW !EX";
		        dirty = true;
		}
                break;


            default:
		if (!dirty)
		{
		        ss << "MMW !M 0 0";
		        dirty = false;
		}
        }

	ss.seekg(0,std::ios::end);
	int size = ss.tellg();
	ss.seekg(0,std::ios::beg);
	if ((dirty) && (size > 3))
	{
		cmdVel_.data = ss.str();
		ss.str("");
	//	ROS_INFO("Send control command [ %s]", cmdVel_.data.c_str());
		pub_.publish(cmdVel_);
	}
    }

}

