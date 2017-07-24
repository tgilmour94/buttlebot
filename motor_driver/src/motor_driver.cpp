#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
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
#define KARTRADIUS      271.4625
#define VOLTAGEYAWREAD  0xE1
#define ODOMREAD        0xE8


int getCheckSum(volatile unsigned char *data, int length);

serial::Serial ser;
int linearVelocity = 0;
int angularVelocity = 0;
int beepcount = 0;
std_msgs::Float32 voltage;

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
  linearVelocity = (int)(vel.linear.x*1000.0);
  angularVelocity = (int)(-vel.angular.z*KARTRADIUS);
}

void CallBackBeeper(const std_msgs::UInt8& msg)
{
 beepcount = msg.data;
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
  unsigned char cmdGetData[8];
  unsigned char cmdSend[] = { 0x55, 0xAB, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF7 };


  // Code Space
  ros::init(argc,argv,"motor_driver");
  ros::NodeHandle nh;
  ros::Subscriber velSub = nh.subscribe("velocity_commands", 1, CallBackMotorControl);
  ros::Subscriber beepSub = nh.subscribe("beep_command", 1, CallBackBeeper);
  ros::Publisher voltPub = nh.advertise<std_msgs::Float32>("battery_voltage",1);
  if(!serialSetup(&cmdSetup[0])) return 0;
  ros::Rate rate (10);

  while(ros::ok())
  {
    //Update Linear & Angular Velocity
    int linGoal;
    int angGoal;
    if(linearVelocity > 0)
    {
      linGoal = 65535 - linearVelocity;
      angGoal = 65535 - angularVelocity;
    }
    else 
    {
      linGoal = -linearVelocity;
      angGoal = -angularVelocity;
    }
    cmdSend[LINVELL] = 0xFF & linGoal;
    cmdSend[LINVELH] = 0xFF & (linGoal >> 8);
    cmdSend[ANGVELL] = 0xFF & angGoal;
    cmdSend[ANGVELH] = 0xFF & (angGoal >> 8);
    //END Update Linear& Angular Velocity
    //Update beeper status
    if(beepcount > 0)
    {
      cmdSend[CTL_BYTE] = 0x08;
      beepcount--;
    }
    else cmdSend[CTL_BYTE] = 0x00;
    


    //Send Command
    cmdSend[XOR] = getCheckSum(&cmdSend[0], 12);
    ser.write(&cmdSend[0], 13);
    
    //Read incoming data
    ser.read(&cmdGetData[0], 8);

    if(cmdGetData[1] == VOLTAGEYAWREAD) //if voltage + yaw info
    { 
      voltage.data = ((cmdGetData[5] <<8)+cmdGetData[6])/1000.0;
      voltPub.publish(voltage);
      float yawinfo = ((cmdGetData[3] <<8)+cmdGetData[4])/1000.0;
      ROS_INFO("%f",yawinfo);

    }
    else if(cmdGetData[1] == ODOMREAD) //if odom info
    {
      float odominfo = ((cmdGetData[3] <<24) + (cmdGetData[4] <<16) + (cmdGetData[5] <<8) + cmdGetData[6])/1000.0;
      ROS_INFO("%f",odominfo);
    } 
    ros::spinOnce();
    rate.sleep();
  }
  return 0;

}

int getCheckSum(volatile unsigned char *data, int length) {

	char XorVal = 0;
	for (int i = 0; i < length; i++) {
		XorVal ^= *data++;
	}
	return XorVal;
}
