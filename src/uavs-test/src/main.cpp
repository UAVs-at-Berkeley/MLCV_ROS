//Standard Include
#include "ros/ros.h"
#include <cstdlib>
#include <string>

//Include Message Types Used
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

//Defines
#define ROLL 0
#define PITCH 1
#define THROTTLE 2
#define YAW 3
#define LEFT_TRIGGER 4

#define PRELIMINARY_CHECK 1
#define LAND_CHECK 2

//throttle values
#define MIDDLE 1500
#define RISING 1690
#define RELEASE 1100
#define NO_RC 900

using namespace std;


int offSpeed [8] = {900,900,900,900,900,900,900,900};
mavros_msgs::RCIn current_rc;
bool rc_available;

/*************** Callbacks  *************/


class Receiver {
public:
//Callbacks
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void vfrCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg);
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg);

    void keyCallback(const geometry_msgs::Twist::ConstPtr& msg);

    bool state_finished;
    bool vfr_finished;
    bool rc_finished;
    bool terminate;

    int state_check;
    int vfr_check;
    double alt_ground;
    int rc_check_ch;
    int rc_check_val;


};

void Receiver::stateCallback(const mavros_msgs::State::ConstPtr& msg) 
{
    if(terminate) return;
    if(state_finished) return;

    bool check1, check2;

    if(state_check == PRELIMINARY_CHECK)
    {
        if(msg->mode == "ALT_HOLD")
        {
           ROS_INFO("COMPLETED: ALT_HOLD");
           check1 = true;
       } 
       else 
       {	
        ROS_INFO("WAITING: ALT_HOLD");
        check1 = false;
        system("rosrun mavros mavsys mode -c ALT_HOLD");
    }

    if(msg->armed)
    {
        ROS_INFO("COMPLETED: Arming");
        check2 = true;
    } else 
    {
        ROS_INFO("WAITING: Arming");
        check2 = false;
        system("rosrun mavros mavsafety arm");
    }

    if(check1 && check2)
    {
        ROS_INFO("COMPLETED: Preliminary Check");
        state_finished = true;
    }	

}
}

void Receiver::vfrCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
    if(terminate) return;

    ROS_INFO_STREAM("Altitude: " << msg->altitude);

    if(vfr_finished) return;

    if (msg->altitude > 0.5) vfr_finished = true;

}


void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
    current_rc = *msg;
    rc_available = true;
}

void Receiver::rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {    if(msg->channels[LEFT_TRIGGER] > 1200) terminate = true;

    rc_cb(msg);
    if(rc_finished) return;

    if(msg->channels[rc_check_ch] <= (rc_check_val + 7) || msg->channels[rc_check_ch] >= (rc_check_val - 7))
    {
	rc_finished = true; //Stop if RC Value is closed to required
	ROS_INFO("Completed: RC Change");    
} 
else rc_finished = false;
}

void Receiver::keyCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
    if (msg->linear.z < 0) {
       ROS_WARN("Received Quit Message");	
   }
}

//UTILITIES

void set_rc(ros::Publisher rc_message, int speed[]) {
    if (!rc_available) {
        ROS_ERROR("Cannot set speed yet");
        return;
    }

    mavros_msgs::OverrideRCIn rc_command;

    for(int i=0; i < 8; i++) rc_command.channels[i] = speed[i];

        rc_message.publish(rc_command);

    bool done = false;
    while (ros::ok() && !done)
    {
        for (int i=0; i < 8; i++) {
            if (current_rc.channels[i] <= (speed[i] + 7) || current_rc.channels[i] >= (speed[i] - 7))
                done = true;
        }
        rc_message.publish(rc_command);
        ros::spinOnce();

    }
}

void key_cb(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "q") {
        throw -1;
    }
}

/*************** MAIN PROGRAM *************/

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    ROS_INFO("MAIN MLCV PROGRAM STARTED");

    Receiver receiver;
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1, &Receiver::stateCallback, &receiver);
    ros::Subscriber vfr_sub = nh.subscribe("/mavros/vfr_hud", 1, &Receiver::vfrCallback, &receiver);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, &Receiver::rcCallback, &receiver);
// ros::Subscriber key_sub = nh.subscribe("/cmd_vel", 1, &Receiver::keyCallback, &receiver);
    ros::Subscriber key_sub = nh.subscribe("/key",1,key_cb);
    receiver.state_finished = true;
    receiver.vfr_finished = true;
    receiver.rc_finished = true;
    receiver.terminate = false;

    ros::Publisher rc_message = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1, true);
    mavros_msgs::OverrideRCIn rc_command;

    try {   
//ARMING
        ROS_INFO("Commencing: Preliminary Setup");

        system("rosrun mavros mavsys mode -c ALT_HOLD");

        system("rosrun mavros mavsafety arm");

        receiver.state_check = PRELIMINARY_CHECK;
        receiver.state_finished = false;
        while((!receiver.state_finished) && (ros::ok()) && (!receiver.terminate)) ros::spinOnce();

        ROS_INFO("Commencing:Take Off");

    //throttle up

        for(int i=0; i < 8; i++) rc_command.channels[i] = NO_RC; 
    //release
            rc_command.channels[THROTTLE] = RELEASE; 
    //ascend

        receiver.rc_check_ch = THROTTLE;
        receiver.rc_check_val = RELEASE;
        receiver.rc_finished = false;

        rc_message.publish(rc_command);
        while((!receiver.rc_finished) && (ros::ok()) && (!receiver.terminate))
        {
            ros::spinOnce();
     	//rc_message.publish(rc_command);
        }

        receiver.vfr_finished = false;    
        while(!receiver.vfr_finished && ros::ok() && !receiver.terminate) 
        {
            ros::spinOnce();
        }

    //RELEASE    
        for(int i=0; i < 8; i++) rc_command.channels[i] = NO_RC; 
        //Release everything

            receiver.rc_check_ch = THROTTLE;
        receiver.rc_check_val = NO_RC;
        receiver.rc_finished = false;
        rc_message.publish(rc_command);
        while((!receiver.rc_finished) && (ros::ok()) && (!receiver.terminate))
        {
            ros::spinOnce();
        //rc_message.publish(rc_command);
        }
        ROS_INFO("RELEASING");

    } catch(int d) {
        if (d==-1) { 
            ROS_INFO("ABORTING MANUALLY");
        }
        set_rc(rc_message, offSpeed);
    }
}
