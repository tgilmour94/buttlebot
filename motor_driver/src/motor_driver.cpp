#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <serial/serial.h>
#include <ros/console.h>

#define	SOP		0
#define CMD		1
#define LEN		2
#define CTL_BYTE	3
#define ANGVELH		6
#define ANGVELL		7
#define LINVELH		8
#define LINVELL		9
#define XOR		12
#define KARTRADIUS      275

int getCheckSum(volatile unsigned char *data, int length);

serial::Serial ser;
int linearVelocity = 0;
int angularVelocity = 0;

void CallBackMotorControl(const geometry_msgs::Twist& vel)
{
/*
linvelH is msb
linvelL is lsb
0x0000 = no speed 
0x0001 = min backward speed
0x0000 + (speed in mm/s converted to hex) = backward speed 
0xffff - (speed in mm/s converted to hex) = forward speed 
*/
  linearVelocity = (int)vel.linear.x;
  angularVelocity = -vel.angular.z*KARTRADIUS;
}

void CallBackBeeper(const std_msgs::Empty& msg)
{
 // ser.write(BEEPERCODE);
}

int serialSetup(unsigned char *data)
{
  try
  {
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
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

	unsigned char cmdSetup[] = { 0x55, 0xac, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0};
	unsigned char cmdGetData[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };
	unsigned char cmdSend[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };

  // Code Space
  ros::init(argc,argv,"motor_driver");
  ros::NodeHandle nh;
  ros::Subscriber velSub = nh.subscribe("velocity_commands", 1, CallBackMotorControl);
  ros::Subscriber beepSub = nh.subscribe("beeper", 10, CallBackBeeper);
  if(!serialSetup(&cmdSetup[0])) return 0;
  ros::Rate rate (10);

  while(ros::ok())
  {
    //Update Linear & Angular Velocity
    int linQuotient;
    int angQuotient;
    if(linearVelocity > 0)
    {
      linQuotient = 65535 - linearVelocity;
      angQuotient = 65535 - angularVelocity;
    }
    else 
    {
      linQuotient = -linearVelocity;
      angQuotient = -angularVelocity;
    }
    unsigned char linTempChar = 0x00;
    unsigned char angTempChar = 0x00;
    for(int i=0;i<4;i++)
    {
      int linTemp = linQuotient%16;
      int angTemp = angQuotient%16;
      switch(i)
      {
        case 0:
        cmdSend[ANGVELL] = 0x00 + angTemp;
        cmdSend[LINVELL] = 0x00 + linTemp;
        break;

        case 1:
        linTempChar = linTemp;
        linTempChar <<=4;
        cmdSend[LINVELL] = linTempChar | cmdSend[LINVELL];
        angTempChar = angTemp;
        angTempChar <<=4;
        cmdSend[ANGVELL] = angTempChar | cmdSend[ANGVELL];       
        break;

        case 2:  
        cmdSend[LINVELH] = 0x00 + linTemp;
        cmdSend[ANGVELH] = 0x00 + angTemp;
        break;
     
        case 3:
        linTempChar = linTemp;
        linTempChar <<=4;
        cmdSend[LINVELH] = linTempChar | cmdSend[LINVELH]; 
        angTempChar = angTemp;
        angTempChar <<=4;
        cmdSend[ANGVELH] = angTempChar | cmdSend[ANGVELH];
        break;
      }
      linQuotient = linQuotient/16;
      angQuotient = angQuotient/16;
    } 
   // ROS_INFO("%d, %d",cmdSend[LINVELH],cmdSend[LINVELL]);
    //END Update Linear& Angular Velocity


    //Send Command
    cmdSend[XOR] = getCheckSum(&cmdSend[0], 12);
    ser.write(&cmdSend[0], 13);
    ros::spinOnce();
    rate.sleep();
  }
  //ser.write(EXITCODE);
  return 0;

}

int getCheckSum(volatile unsigned char *data, int length) {

	char XorVal = 0;
	for (int i = 0; i < length; i++) {
		XorVal ^= *data++;
	}
	return XorVal;
}
