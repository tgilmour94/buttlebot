#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sstream>

//#define ROSINFO
  int lowH = 55;
  int highH = 105;
  int lowS = 0;
  int highS = 255;
  int lowV = 0;
  int highV = 255;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  
protected:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber rgb_sub;
  image_transport::Subscriber depth_sub;
  sensor_msgs::ImageConstPtr rgbImageIn_;
  sensor_msgs::ImageConstPtr depthImageIn_;
  cv_bridge::CvImagePtr rgbCvPtr;
  cv_bridge::CvImagePtr depthCvPtr;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    rgb_sub = it_.subscribe("/camera/rgb/image_rect_color", 1, 
      &ImageConverter::rgbImageCb, this,image_transport::TransportHints("raw"));
    depth_sub = it_.subscribe("/camera/depth_registered/image_raw", 1,       	&ImageConverter::depthImageCb, this, image_transport::TransportHints("raw"));

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow("controller");
    cv::createTrackbar("Hue Lower Bound", "controller", &lowH, highH);
    cv::createTrackbar("Hue Upper Bound", "controller", &highH, highH);
    cv::createTrackbar("Sat Lower Bound", "controller", &lowS, highS);
    cv::createTrackbar("Sat Upper Bound", "controller", &highS, highS);
    cv::createTrackbar("Value Lower Bound", "controller", &lowV, highV);
    cv::createTrackbar("Value Upper Bound", "controller", &highV, highV);
  }

  ~ImageConverter() //~ means bitwise NOT so when imageconverter object is closed
  {
    cv::destroyAllWindows();
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
    //cv::GaussianBlur(depthCvPtr->image,depthCvPtr->image,cv::Size(13,13),0,0);

    double aveDepth = 0;
    int count = 0;
    unsigned short max = 0;
    for(int i=0; i<(HSV.cols-1);i++)
    {
      for(int j=0; j<(HSV.rows-1);j++)
      {
        if(HSV.at<uchar>(cv::Point(i,j)) == 255)
        {
          if (depthCvPtr->image.at<unsigned short>(cv::Point(i,j)) > max)
            max = depthCvPtr->image.at<unsigned short>(cv::Point(i,j));
          if (depthCvPtr->image.at<unsigned short>(cv::Point(i,j)) != 0)
	  {
            aveDepth += depthCvPtr->image.at<unsigned short>(cv::Point(i,j));
            count++;
          }
        }
      }
    }
    std::ostringstream os;
    if (count == 0) count =1;
    int depth = aveDepth/count;
    os << "depth: " << depth << " mm " ; 
    cv::putText(HSV, os.str() ,cv::Point(rgbCvPtr->image.cols/4, rgbCvPtr->image.rows/8),1,1, cv::Scalar(255,255,255),2); 
    
    double minVal, maxVal;
   // cv::minMaxLoc(depthCvPtr->image, &minVal, &maxVal);
   // depthCvPtr->image.convertTo(depthCvPtr->image, CV_8U, 255/(maxVal-minVal) ,-minVal * 255.0/(maxVal - minVal));
   // imshow("depth", depthCvPtr->image);
    imshow("HSV", HSV);
    imshow(OPENCV_WINDOW, rgbCvPtr->image);
    cv::waitKey(1);
  }

  virtual void spin()
  {
    ros::Rate rate (30);
    while (ros::ok())
    {
      spinOnce();
      rate.sleep();
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
  ros::init(argc, argv, "test");
  ImageConverter ic;
  ic.spin();
}
