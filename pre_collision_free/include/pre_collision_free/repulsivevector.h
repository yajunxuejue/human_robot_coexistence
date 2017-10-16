#ifndef REPULSIVEVECTOR_H
#define REPULSIVEVECTOR_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Eigen>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/camera_subscriber.h>

class RepulsiveVector1
{
//private:

  //Eigen::Vector4f controlPointsDepthcoordinate;


  //image_transport::Publisher image_pub_;
 // ros::Subscriber joint_states_sub_;
public:
//   int flag;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;

public:
  RepulsiveVector1(ros::NodeHandle &nh);
  ~RepulsiveVector1();
  void repulsiveVectorCallback();
//  void getControlPointsCallback();
//  void getRepulsiveVector();
};


#endif // REPULSIVEVECTOR_H
