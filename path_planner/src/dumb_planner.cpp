#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Empty.h>
#include <ros/console.h>

//#define PIDCHANGER // allows you to change PID values of the motion planner in order to test smooth response

#ifdef PIDCHANGER
#include <std_msgs/Float32.h>
float LINKP = 1.00;
float ANGKP = 1.5;

#else

#define LINKP 1.0 //1.25
#define ANGKP 1.5 //2.0

#endif

#define GOALDISTANCE 1.0
#define GOALANGLE 0.0
#define THRESHANGLEHIGH 0.175 // +-10 degrees
#define THRESHANGLELOW -0.175
#define THRESHHIGH 1.2 //metres
#define THRESHLOW 0.8
#define MINERROR 0.01 //10cm
#define MINANGERROR 0.04 //3 degrees
#define FOLLOW 2
#define WAIT 3

const ros::Duration TIMEOUT(0.5);

float objectDistance = 1.0;
float objectLocation = 0.0;
ros::Time lastGoalTime;

void CallBackGoalLocation(const geometry_msgs::Vector3Stamped& goal)
{
 objectDistance = goal.vector.x;
 objectLocation = goal.vector.z;
 lastGoalTime = ros::Time::now();
}
#ifdef PIDCHANGER
void LinPIDCallback(const std_msgs::Float32& LKP)
{
  LINKP = LKP.data;
}

void AngPIDCallback(const std_msgs::Float32& AKP)
{
  ANGKP = AKP.data;
}
#endif

int main(int argc, char **argv)
{
  ros::init(argc,argv,"dumb_follower");
  ros::NodeHandle nh;
  ros::Subscriber goalSub = nh.subscribe("target_location", 1, CallBackGoalLocation);
  ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("velocity_commands",1);
#ifdef PIDCHANGER
  ros::Subscriber linkpSub = nh.subscribe("new_lin_kp", 1, LinPIDCallback);
  ros::Subscriber angkpSub = nh.subscribe("new_ang_kp", 1, AngPIDCallback);

#endif

  ros::Rate rate (30);
  lastGoalTime = ros::Time::now();
  
  
  while(ros::ok())
  {
    static int state = WAIT;
    if( ((ros::Time::now() - lastGoalTime) > TIMEOUT) && state != WAIT)
    {
      state = WAIT;
    //  ROS_INFO("TIMEOUT");
      geometry_msgs::Twist curVelocity;
      curVelocity.linear.x = 0;
      curVelocity.angular.z = 0;
      velPub.publish(curVelocity);
     
    }
    if(state == FOLLOW)
    {
	//linear velocity controller
      float curLinError = objectDistance - GOALDISTANCE;
      float linearVelocity = (curLinError*LINKP);
	//end linear velocity controller

	//angular velocity controller 
      float curAngError = objectLocation - GOALANGLE;
      float angularVelocity = curAngError*ANGKP; 
	//end angular velocity controller

      geometry_msgs::Twist curVelocity;
      curVelocity.linear.x = linearVelocity;
      curVelocity.angular.z = -angularVelocity;
      velPub.publish(curVelocity);
      if(fabs(curLinError) <= MINERROR && fabs(curAngError) <= MINANGERROR)
      {
      //  ROS_INFO("GOALREACHED");
        state = WAIT;
        curVelocity.linear.x = 0;
	curVelocity.angular.z = 0;
	velPub.publish(curVelocity);
      }   
    }

    else if(state == WAIT) //wait because of timeout or because goal has been reached
    {

      if((objectDistance > THRESHHIGH || objectDistance < THRESHLOW || objectLocation > THRESHANGLEHIGH || objectLocation < THRESHANGLELOW) && ((ros::Time::now() - lastGoalTime) < TIMEOUT))
      {
        state = FOLLOW;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;

}

