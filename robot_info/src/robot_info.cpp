#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#define INTZERO 0
#define FLOATZERO 0.0
#define MAXMANUALLINVEL 0.4 //400mm/s max speed
#define MAXMANUALANGVEL 1.0
#define LOWVOLTAGE 34.0 //
#define LOW 0
#define HIGH 1

class RobotInfo
{

public:
  ros::NodeHandle nh;

  ros::Subscriber levelSub;
  ros::Subscriber yawSub;
  ros::Subscriber pitchSub;
  ros::Subscriber rollSub;
  ros::Subscriber heightSub;
  ros::Subscriber followSub;
  ros::Subscriber manCtrlSub;
  ros::Subscriber voltageSub;

  ros::Publisher mcuPub;
  ros::Publisher followPub;
  ros::Publisher manCtrlPub;
  ros::Publisher beepPub;

  std_msgs::Float32MultiArray moduleStatus;
  std_msgs::Float32 levelGoal;
  std_msgs::Float32 yawGoal;
  std_msgs::Float32 pitchGoal;
  std_msgs::Float32 rollGoal;
  std_msgs::Float32 heightGoal;
  std_msgs::UInt8 autoFollowGoal; 
  geometry_msgs::Twist manCtrlGoal;
  int batteryState;
  


  RobotInfo()
  { 
    //Subscribers to kart commands from GUI
    levelSub = nh.subscribe("self_level_control", 1, &RobotInfo::CallBackSelfLevel, this);
    yawSub = nh.subscribe("yaw_goal", 1, &RobotInfo::CallBackYaw, this);
    pitchSub = nh.subscribe("pitch_goal", 1, &RobotInfo::CallBackPitch, this);
    rollSub = nh.subscribe("roll_goal", 1, &RobotInfo::CallBackRoll, this);
    heightSub = nh.subscribe("height_goal", 1, &RobotInfo::CallBackHeight, this);
    followSub = nh.subscribe("auto_follow_control", 1, &RobotInfo::CallBackAutoFollow, this);
    //subscriber to kart information
    manCtrlSub = nh.subscribe("joystick_vel", 1, &RobotInfo::CallBackJoystickControl, this);
    voltageSub = nh.subscribe("battery_voltage", 1, &RobotInfo::CallBackBatteryVoltage, this);
    
    //publishers to kart & MCU to fulfill commands
    mcuPub = nh.advertise<std_msgs::Float32MultiArray>("module_command",1);
    followPub = nh.advertise<std_msgs::UInt8>("follow_command",1);
    manCtrlPub = nh.advertise<geometry_msgs::Twist>("velocity_commands", 1);
    beepPub = nh.advertise<std_msgs::UInt8>("beep_command",1);
 
    moduleStatus.layout.dim.push_back(std_msgs::MultiArrayDimension());
    moduleStatus.layout.dim[0].label = "data";
    moduleStatus.layout.dim[0].size = 5;
    moduleStatus.layout.dim[0].stride = 5;

    levelGoal.data = FLOATZERO;
    yawGoal.data = FLOATZERO;
    pitchGoal.data = FLOATZERO;
    rollGoal.data = FLOATZERO;
    heightGoal.data = FLOATZERO;
    autoFollowGoal.data = INTZERO;
    batteryState = HIGH;

  }

  ~RobotInfo() 
  {
  }

  void CallBackSelfLevel(const std_msgs::Float32& commandLevel)
  {
    levelGoal.data = commandLevel.data;
  }

  void CallBackYaw(const std_msgs::Float32& commandYaw)
  {
    yawGoal.data = commandYaw.data;
  }

  void CallBackPitch(const std_msgs::Float32& commandPitch)
  {
    pitchGoal.data = commandPitch.data;
  }

  void CallBackRoll(const std_msgs::Float32& commandRoll)
  {
    rollGoal.data = commandRoll.data;
  }

  void CallBackHeight(const std_msgs::Float32& commandHeight)
  {
    heightGoal.data = commandHeight.data;
  }

  void CallBackAutoFollow(const std_msgs::UInt8& commandFollow)
  {
    if(batteryState != LOW)
    {
      autoFollowGoal.data = commandFollow.data;
      followPub.publish(autoFollowGoal);
    }
  }
  
  void CallBackJoystickControl(const geometry_msgs::Twist& vel)
  {
    if(batteryState != LOW && autoFollowGoal.data == LOW)
    {
      manCtrlGoal.linear.x = MAXMANUALLINVEL*vel.linear.x;
      manCtrlGoal.angular.z = MAXMANUALANGVEL*vel.angular.z;
      manCtrlPub.publish(manCtrlGoal);
    }
  }
  
  void CallBackBatteryVoltage(const std_msgs::Float32& voltage)
  {
    if(voltage.data <= LOWVOLTAGE)
    {
      batteryState = LOW;
      manCtrlGoal.linear.x = 0.0;
      manCtrlGoal.angular.z = 0.0;
      manCtrlPub.publish(manCtrlGoal);
      autoFollowGoal.data = 0;
      followPub.publish(autoFollowGoal);
      moduleStatus.data.clear();
      mcuPub.publish(moduleStatus); 
      std_msgs::UInt8 beeper;
      beeper.data = 20;
      beepPub.publish(beeper);    
    }
  }

  void process()
  { 
    if(batteryState != LOW)
    {
      moduleStatus.data.clear();
      moduleStatus.data.push_back(levelGoal.data);
      moduleStatus.data.push_back(yawGoal.data);
      moduleStatus.data.push_back(pitchGoal.data);
      moduleStatus.data.push_back(rollGoal.data);
      moduleStatus.data.push_back(heightGoal.data);

      mcuPub.publish(moduleStatus);
    }
  }

  virtual void spinOnce()
  {
    process();
    ros::spinOnce();
  }     
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_info_class");
  RobotInfo RI;
  ros::Rate rate (10);
  while (ros::ok())
  {
    RI.spinOnce();
    rate.sleep();
  }
}
