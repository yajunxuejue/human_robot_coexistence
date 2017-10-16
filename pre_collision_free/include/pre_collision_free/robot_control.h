#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

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

#include <tf/transform_listener.h>

#include <trac_ik/trac_ik.hpp>
#include "geometry_msgs/Pose.h"
#include <kdl_conversions/kdl_msg.h>

//#include <moveit/move_group_interface/move_group.h>

class RepulsiveVector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  tf::TransformListener listener_;
  ros::Subscriber joint_states_sub_;
  uint region_surveillance_width, region_surveillance_height;
  bool flag_calIK;
  cv::Mat cv_flitered_depth_image;

  ros::Publisher joint_position_pub;

  KDL::JntArray ll, ul;
  //geometry_msgs::Pose pose_c;

  //Eigen::Vector3f repulsive_vector, sum_repulsive_vector;

public:
  RepulsiveVector(ros::NodeHandle &nh);
  ~RepulsiveVector();
  void repulsiveVectorCallback(const sensor_msgs::ImageConstPtr& msgs);
  void jointStateCallback(const sensor_msgs::JointState _jointstate);
  void getControlPoints();
  void calculateRepulsiveVector();
  //void robotControl(Eigen::Vector3f sum_repulsive_vector_);
  void calculateIK(KDL::Frame desired_end_effector_pose);
  Eigen::Matrix3f cam_rgb_tf_base_rot;
  KDL::JntArray nominal;
  void test_trac_ik();
  float tool0_control_point_x, tool0_control_point_y, tool0_control_point_dz;
};
#endif // ROBOT_CONTROL_H
