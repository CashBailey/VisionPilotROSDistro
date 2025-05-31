#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat receivedImage = cv_bridge::toCvShare(msg, "bgr8")->image; // Save image in a Mat object
    cv::imwrite("/home/noe/catkin_ws/src/offboard/image_reference/reference_image.jpg",receivedImage);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_image_listener");
    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);

    std::string topicName;
    if(argc != 1){
      topicName = argv[1];
    } 
    else
    {
      topicName = "/my_image";
      std::cout <<"No topic name given. The default topic name will be used: " << topicName << std::endl;
    }

    
    image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);
    ros::spin();
    //cv::destroyWindow("Video Stream");
    return 0;
}