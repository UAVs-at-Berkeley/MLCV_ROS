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

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

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



mavros_msgs::State current_state;
mavros_msgs::RCIn current_rc;

bool rc_available = false;
// ROLL, PITCH, THROTTLE, YAW, LEFT_TRIGGER
int onSpeed [8] = {900,900,1100,900,900,900,900,900};
int midSpeed [8] = {1200,900,1100,900,900,900,900,900};
int offSpeed [8] = {900,900,900,900,900,900,900,900};

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg) 
{
	current_rc = *msg;
	rc_available = true;
}

void key_cb(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data == "q") {
		throw -1;
	}
}

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

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "ExpNode");
	ros::NodeHandle nh;
	
	
	ros::Publisher rc_message = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1, true);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, rc_cb);	
	ros::Subscriber key_sub = nh.subscribe("/key",1,key_cb);
	
	ros::Rate rate(20.0);

	try {

		//wait til start
		while(ros::ok() && !current_state.connected) {
			ros::spinOnce();
			rate.sleep();
		}

		ros::Duration flyDuration(2.);

		mavros_msgs::OverrideRCIn rc_command;

		//arming

		mavros_msgs::CommandBool arm_cmd;
		arm_cmd.request.value = true;

		mavros_msgs::SetMode set_guided_cmd;
		set_guided_cmd.request.custom_mode = "STABILIZE";

		while(ros::ok() && !(mode_client.call(set_guided_cmd) && set_guided_cmd.response.success)) {
			ros::spinOnce();
			rate.sleep();
		}
		ROS_INFO("Mode Changed to STABILIZE");
		while(ros::ok() && !(arming_client.call(arm_cmd) && arm_cmd.response.success)) {
			ros::spinOnce();
			rate.sleep();
		}
		ROS_INFO("Quad Armed");	
		
		while (!rc_available) 
		{
			ros::spinOnce();
		}

		//experimental stuff
		ROS_WARN("Beginning Tests");	

		set_rc(rc_message,onSpeed);   

		flyDuration.sleep();

		set_rc(rc_message,midSpeed);

		flyDuration.sleep();

		set_rc(rc_message,offSpeed); 	 

		ROS_INFO("EXITING");   

	} catch (int d) {
		if (d==-1) {
			ROS_INFO("ABORTED MANUALLY");
		}
		set_rc(rc_message, offSpeed);
	}
}
