#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Empty.h>
#include <ros/console.h>

#define GOALDISTANCE 1.0
#define GOALANGLE 0.0
#define THRESHHIGH 1.2
#define THRESHLOW 0.8
#define LINKP 1.25
#define ANGKP 2.0
#define MINERROR 0.01
#define FOLLOW 2
#define WAIT 3

const ros::Duration TIMEOUT(0.5);

float objectDistance;
float objectLocation;
ros::Time lastGoalTime;

void CallBackGoalLocation(const geometry_msgs::Vector3Stamped& goal)
{
 objectDistance = goal.vector.x;
 objectLocation = goal.vector.z;
 lastGoalTime = ros::Time::now();
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"dumb_follower");
  ros::NodeHandle nh;
  ros::Subscriber goalSub = nh.subscribe("target_location", 1, CallBackGoalLocation);
  ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("velocity_commands",1);
  ros::Rate rate (30);
  lastGoalTime = ros::Time::now();
  static int state = FOLLOW;
  
  while(ros::ok())
  {
    if( (ros::Time::now() - lastGoalTime) > TIMEOUT)
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
      curVelocity.angular.z = angularVelocity;
      velPub.publish(curVelocity);
      if(fabs(curLinError) <= MINERROR)
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
      if((objectDistance > THRESHHIGH || objectDistance < THRESHLOW) && ((ros::Time::now() - lastGoalTime) < TIMEOUT))
      {
        state = FOLLOW;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;

}

