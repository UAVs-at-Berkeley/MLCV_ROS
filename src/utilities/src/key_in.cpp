#include <ros/ros.h>

#include <std_msgs/String.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sstream>
#include <string>

using namespace std;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "key_in");   
ros::NodeHandle nh;
ros::Publisher keyPub = nh.advertise<std_msgs::String>("/key",1);

ros::Rate rate(5.);

ROS_INFO("STARTING KEYBOARD BROADCASTING");

while (ros::ok())
{
char c = getch();
stringstream ss;
string s;
ss << c;
ss >> s;
std_msgs::String msg;
msg.data = s;
keyPub.publish(msg);
rate.sleep();
}

}

