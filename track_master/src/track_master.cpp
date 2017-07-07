#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <sstream>

//#define ROSINFO
//#define VISUAL
#define WAIT 0
#define FOLLOW 1
#define PIX2RADS 0.00159534

#ifdef VISUAL
 int lowH = 0;
 int highH = 255;
 int lowS = 0;
 int highS = 255;
 int lowV = 0;
 int highV = 255;
#else
 int lowH = 58;
 int highH = 97;
 int lowS = 66;
 int highS = 168;
 int lowV = 65;
 int highV = 184;
#endif

class ImageConverter
{
  
protected:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgbSub;
  image_transport::Subscriber depthSub;
  sensor_msgs::ImageConstPtr rgbImageIn_;
  sensor_msgs::ImageConstPtr depthImageIn_;
  cv_bridge::CvImagePtr rgbCvPtr;
  cv_bridge::CvImagePtr depthCvPtr;

  ros::Publisher goalPub;
  ros::Subscriber followSub;
  std_msgs::UInt8 followState;
  geometry_msgs::Vector3Stamped goal;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    rgbSub = it_.subscribe("/camera/rgb/image_rect_color", 1, 
    &ImageConverter::rgbImageCb, this,image_transport::TransportHints("raw"));
    depthSub = it_.subscribe("/camera/depth_registered/image_raw", 1,
    &ImageConverter::depthImageCb, this,image_transport::TransportHints("raw"));
    followSub = nh_.subscribe("follow_command",1,&ImageConverter::followCb, this);

    goalPub = nh_.advertise<geometry_msgs::Vector3Stamped>("target_location", 1);
    followState.data = WAIT;
   
#ifdef VISUAL
    cv::namedWindow("Image window");
    cv::namedWindow("controller");
    cv::createTrackbar("Hue Lower Bound", "controller", &lowH, highH);
    cv::createTrackbar("Hue Upper Bound", "controller", &highH, highH);
    cv::createTrackbar("Sat Lower Bound", "controller", &lowS, highS);
    cv::createTrackbar("Sat Upper Bound", "controller", &highS, highS);
    cv::createTrackbar("Value Lower Bound", "controller", &lowV, highV);
    cv::createTrackbar("Value Upper Bound", "controller", &highV, highV);
#endif
  }

  ~ImageConverter() 
  {
#ifdef VISUAL
    cv::destroyAllWindows();
#endif
  }

  void rgbImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    rgbImageIn_ = msg;
#ifdef ROSINFO
    ROS_INFO("rgb image callback");
    if (rgbImageIn_==NULL)  ROS_INFO("NULL FILLER");
#endif
    
  }
 
  void depthImageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    depthImageIn_ = msg;
#ifdef ROSINFO
    ROS_INFO("depth image callback");
    if (depthImageIn_==NULL)  ROS_INFO("depth NULL FILLER");
#endif
  }
  
  void followCb(const std_msgs::UInt8& command)
  { 
    followState.data = command.data;
  }

  void process()
  {
//you may notice that before processing the data, I copy the shared pointer. 
//This is to ensure that, if you use an ASyncSpinner, your image won't change in the 
//middle of the processing. Shared pointers copies are atomic operations which means 
//that this callback is thread-safe. This is required to convert this node to a 
//nodelet for instance and also to deal with more complex GUI that may require you to 
//use the ASyncSpinner. Note that no mutex are needed here, even in this case.
    sensor_msgs::ImageConstPtr depthImageIn = depthImageIn_;
    sensor_msgs::ImageConstPtr rgbImageIn = rgbImageIn_;
    if(depthImageIn==NULL || rgbImageIn==NULL)
    {
#ifdef ROSINFO
       ROS_INFO("NULL");
#endif
    return;
    }
    
    try
    {
#ifdef ROSINFO
      ROS_INFO("depth CV conversion");
#endif
      depthCvPtr = cv_bridge::toCvCopy(depthImageIn, sensor_msgs::image_encodings::TYPE_16UC1);
    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    try
    {
#ifdef ROSINFO
      ROS_INFO("rgb CV conversion");
#endif
      rgbCvPtr = cv_bridge::toCvCopy(rgbImageIn, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat HSV;
    cv::cvtColor(rgbCvPtr->image, HSV, CV_BGR2HSV);
    cv::inRange(HSV, cv::Scalar(lowH,lowS,lowV),cv::Scalar(highH,highS,highV),HSV);
    erode(HSV,HSV, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11)) );
    dilate(HSV,HSV, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(11,11)) );
    dilate(HSV,HSV, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(9,9)) );
    erode(HSV,HSV, cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(9,9)) );
    cv::medianBlur(depthCvPtr->image,depthCvPtr->image,5);

    double aveDepth = 0;
    double aveLocation = 0;
    int count = 0;
    for(int i=0; i<(HSV.cols-1);i++)
    {
      for(int j=0; j<(HSV.rows-1);j++)
      {
        if(HSV.at<uchar>(cv::Point(i,j)) == 255)
        {
          if (depthCvPtr->image.at<unsigned short>(cv::Point(i,j)) != 0)
	  {
            aveDepth += depthCvPtr->image.at<unsigned short>(cv::Point(i,j));
            aveLocation += (i-320.0);
            count++;
          }
        }
      }
    }

    if (count != 0)
    {
      float depth = (aveDepth/1000.0)/count;
      float location = (aveLocation*PIX2RADS)/count;
      goal.vector.x = depth;
      goal.vector.z = location;
      goalPub.publish(goal);
#ifdef VISUAL
      std::ostringstream os;
      os << "depth: " << depth << " mm " ; 
      cv::putText(HSV, os.str() ,cv::Point(rgbCvPtr->image.cols/4, rgbCvPtr->image.rows/8),1,1, cv::Scalar(255,255,255),2); 
#endif
    }

#ifdef VISUAL
    imshow("HSV", HSV);
    imshow("Image window", rgbCvPtr->image);
    cv::waitKey(1);
#endif
  }

  virtual void spinOnce()
  {
    if(followState.data ==FOLLOW)
    {
      process();
    }

    ros::spinOnce();
  }     
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "track_master");
  ImageConverter ic;
  ros::Rate rate (30);

  while (ros::ok())
  {
    ic.spinOnce();
    rate.sleep();
  }
}
