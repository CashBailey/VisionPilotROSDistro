#include "utils.h"
#include "vision_utils.h"
#include <Eigen/Dense>

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdlib.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <eigen_conversions/eigen_msg.h>

cv::Mat refImage, currentImage;
double t0,t;

static const std::string REF_IMAGE_NAME = "/home/noe/catkin_ws/src/offboard/image_reference/reference_image.jpg";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cvPtr;

	try
	{ 
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
  cvPtr->image.copyTo(currentImage);
}



int main(int argc, char **argv)
{
    Matrix_t K(3,3); //Calibration of the camera matrix
    Matrix_t H(3,3); // Homography matrix
    Matrix_t R(3,3); // Rotation between current frame and the reference frame
    Vector3D_t t;    // Translation between current frame and the reference frame
    Vector3D_t n;    // normal vector in the reference plane
    Vector3D_t u;   // Rodrigues vector
    Vector3D_t Uv;   // Control input for the translational velocity
    Vector3D_t Uw;   // Control input for the rotational velocity

    FPTYPE d;       // Distance between the camera and the reference plane
    homographySolution homSolution;
    int counter = 0;


    K << 554.382713,     0.0   , 320.0, 
         0.0       , 554.382713, 240.0, 
         0.0       ,     0.0   ,   1.0;
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <topic_name> <lambdav> <lambdaw>" << std::endl;
        return 1;
    }

    
    FPTYPE lambdav = atof(argv[2]);
    FPTYPE lambdaw = atof(argv[3]);

    refImage = cv::imread(REF_IMAGE_NAME, cv::IMREAD_COLOR);
    if(refImage.empty())
    {
        std::cout << "Could not read the image: " << REF_IMAGE_NAME << std::endl;
        return 1;
    }
    else{
        std::cout << "Ref image is ready: " << REF_IMAGE_NAME << std::endl;
    }

    ros::init(argc, argv, "pbvs_node");
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

    ros::Publisher homography_pub   = nh.advertise<std_msgs::Float64MultiArray>("/homograpy_numerical",10);
    ros::Publisher ck_t_ct_pub      = nh.advertise<std_msgs::Float64MultiArray>("/ck_t_ct",10);
    ros::Publisher n_pub            = nh.advertise<std_msgs::Float64MultiArray>("/n_plane_vector",10);
    ros::Publisher Uv_pub           = nh.advertise<std_msgs::Float64MultiArray>("/Uv",10);
    ros::Publisher Uw_pub           = nh.advertise<std_msgs::Float64MultiArray>("/Uw",10);
    ros::Publisher d_pub            = nh.advertise<std_msgs::Float64>("/d_value",10);


    image_transport::Subscriber sub = it.subscribe(topicName, 1, imageCallback);

    //ros::spin();
    ros::Rate loop_rate(10); //Se ejecuta 100 por segundo
    ros::Duration(2).sleep(); //Retardo  de tiempo

    std_msgs::Float64MultiArray H_msg,ck_t_ct_msg,n_msg, Uv_msg, Uw_msg;
    std_msgs::Float64 d_msg;

    do{
          if(currentImage.empty())
          {
              std::cout << "Could not read the current image: "  << std::endl;
          }
          else
          {
              //std::cout << "current image is ready: "  << std::endl;
              cv::imwrite("/home/noe/catkin_ws/src/offboard/src/current_image.jpg",currentImage);
              EstimateHomography(refImage, currentImage, K, H, counter );
              RecoverFromHomography(H,R,t,n,d, counter, homSolution);
              Rodriguez(R,u);
              PBVSController(R,t,u,Uv,Uw,lambdav,lambdaw);
              
              tf::matrixEigenToMsg(Uv, Uv_msg);
              tf::matrixEigenToMsg(Uw, Uw_msg);
              tf::matrixEigenToMsg(H, H_msg);
              tf::matrixEigenToMsg(t, ck_t_ct_msg);
              tf::matrixEigenToMsg(n, n_msg);
              homography_pub.publish(H_msg);
              ck_t_ct_pub.publish(ck_t_ct_msg);
              n_pub.publish(n_msg);
              d_msg.data = d;
              d_pub.publish(d_msg);
              Uv_pub.publish(Uv_msg);
              Uw_pub.publish(Uw_msg);
          }
          counter++;
          ros::spinOnce();   //Activa callbacks
          loop_rate.sleep(); //Espera hasta completar el loop rate
    }while(ros::ok());

    //cv::destroyWindow("Video Stream");
    return 0;
}
