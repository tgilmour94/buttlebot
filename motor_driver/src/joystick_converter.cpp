#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

#define MAXLINVEL 400.0 //400mm/s max speed
#define MAXANGVEL 1.0 //1rad/s max rotation

ros::Publisher velPub;

void CallBackJoystickConverter(const geometry_msgs::Twist& vel)
{
  geometry_msgs::Twist vel_command;
  vel_command.linear.x = MAXLINVEL*vel.linear.x;
  vel_command.angular.z = MAXANGVEL*vel.angular.z;
  velPub.publish(vel_command);
  
}


int main(int argc, char **argv)
{
  // Variables

	unsigned char cmdSetup[] = { 0x55, 0xac, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0};
	unsigned char cmdGetData[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };
	unsigned char cmdSend[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };

  // Code Space
  ros::init(argc,argv,"joystick_converter");
  ros::NodeHandle nh;
  ros::Subscriber velSub = nh.subscribe("cmd_vel", 1, CallBackJoystickConverter);
  velPub = nh.advertise<geometry_msgs::Twist>("velocity_commands", 1);

  while(ros::ok())
  {
    ros::spin();
  }
  return 0;

}

