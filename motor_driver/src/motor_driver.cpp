#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <serial/serial.h>
#include <ros/console.h>
//#include <string.h>

serial::Serial ser;
std::string BEGINCODE = "55AC09000000000000000000F0";
std::string EXITCODE = "55AE09000000000000000000";
std::string RTDATACODE = "55AB09000000000000000000F7";

void CallBackMotorControl(const geometry_msgs::Twist& vel)
{
 //  ser.write(MOVECOMMAND);
}
void CallBackBeeper(const std_msgs::Empty& msg)
{
 // ser.write(BEEPERCODE);
}

int serialSetup()
{
  try
  {
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(9600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_INFO("unable to open port");
    return 0;
  }
  if(ser.isOpen())
  {
    ROS_INFO("Serial Port Initialized");
    ser.write(BEGINCODE);
    return 1;
  }
  else return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"motor_driver");
  ros::NodeHandle nh;
  ros::Subscriber velSub = nh.subscribe("velocity_commands", 1, CallBackMotorControl);
  ros::Subscriber beepSub = nh.subscribe("beeper", 10, CallBackBeeper);
  if(!serialSetup()) return 0; 
  ros::Rate rate (10);
  while(ros::ok())
  {
    ser.write(RTDATACODE);
    ros::spinOnce();
    rate.sleep();
  }
  ser.write(EXITCODE);
  return 0;

}
