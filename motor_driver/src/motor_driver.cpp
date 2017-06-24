#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <serial/serial.h>
#include <ros/console.h>
//#include <string.h>

#define	SOP			  0
#define CMD			  1
#define LEN			  2
#define CTL_BYTE	3
#define ANGVELH		6
#define ANGVELL		7
#define LINVELH		8
#define LINVELL		9
#define XOR			  12

int getCheckSum(volatile unsigned char *data, int length);

serial::Serial ser;

void CallBackMotorControl(const geometry_msgs::Twist& vel)
{
 //  ser.write(MOVECOMMAND);
}
void CallBackBeeper(const std_msgs::Empty& msg)
{
 // ser.write(BEEPERCODE);
}

int serialSetup(*data)
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
    ser.write(data, 13);
    return 1;
  }
  else return 0;
}

int main(int argc, char **argv)
{
  // Variables

	unsigned char cmdSetup[] = { 0x55, 0xac, 0x09, 0x00, 0x00, 0x00, 0x00,
						                   0x00, 0x00, 0x00, 0x00, 0x00, 0xF0};
	unsigned char cmdGetData[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00,
								                 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };
	unsigned char cmdSend[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00,
								              0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };

  // Code Space
  ros::init(argc,argv,"motor_driver");
  ros::NodeHandle nh;
  ros::Subscriber velSub = nh.subscribe("velocity_commands", 1, CallBackMotorControl);
  ros::Subscriber beepSub = nh.subscribe("beeper", 10, CallBackBeeper);
  if(!serialSetup(&cmdSetup[0])) return 0;
  ros::Rate rate (10);

  // Test - Turn on motors
  cmdSend[LINVELL] = 0x10;
  cmdSend[LINVELH] = 0x10;

  while(ros::ok())
  {

    cmdSend[XOR] = getCheckSum(&cmdSend[0], 12);
    ser.write(&cmdSend[0], 13);
    ros::spinOnce();
    rate.sleep();
  }
  ser.write(EXITCODE);
  return 0;

}

int getCheckSum(volatile unsigned char *data, int length) {

	char XorVal = 0;
	for (int i = 0; i < length; i++) {
		XorVal ^= *data++;
	}
	return XorVal;
}
