#include "pre_collision_free/repulsivevector.h"

static const std::string OPENCV_WINDOW = "Image window";

RepulsiveVector1::RepulsiveVector1(ros::NodeHandle &nh)
  :nh_(nh)
  ,it_(nh)

{

  //joint_states_sub_=nh_.subscribe("/joint_states", 1000,
  //                                 RepulsiveVector::getControlPointsCallback);

  image_sub_= it_.subscribeCamera("/camera/depth_registered_filtered/image_rect", 10, &RepulsiveVector::repulsiveVectorCallback, this);

  cv::namedWindow(OPENCV_WINDOW);

}

RepulsiveVector1::~RepulsiveVector1()
{
  cv::destroyWindow(OPENCV_WINDOW);
}

void RepulsiveVector1::repulsiveVectorCallback(const sensor_msgs::ImageConstPtr& camera_image)
{
 // if(flag==0)
 // {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat depth_image_flitered;
    try
    {
      cv_ptr = cv_bridge::toCvShare(camera_image, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_image_flitered = cv_ptr->image;
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
//    cv::circle(cv_prt->image, cv::Point(RepulsiveVector::controlPointsDepthcoordinate(0,0),
//                                        RepulsiveVector::controlPointsDepthcoordinate(1,0)),
//                                        5, CV_RGB(255,0,0));
//    cv::Mat imageROI;
//    imageROI = depth_image_flitered(Rect(RepulsiveVector::controlPointsDepthcoordinate(0,0),
//                                         RepulsiveVector::controlPointsDepthcoordinate(1,0),
//                                         50,
//                                         50));


//    flag=1;
//  }else{
//    return;
 // }
}

//void RepulsiveVector::getControlPointsCallback(const sensor_msgs::JointStateConstPtr& robot_joint_position)
//{

//    Eigen::Matrix4f camera_in;
//    Eigen::Matrix3f camera_ex;

//    camera_ex<<0,0,0,0,
//               0,0,0,0,
//               0,0,0,0,
//               0,0,0,0;
//    camera_in<<0,0,0,
//               0,0,0,
//               0,0,0;

//    //controlPointsDepthcoordinate = camera_in * camera_ex;
//    flag = 0;
//    std::cout<<flag<<endl;
//}

//void RepulsiveVector::getRepulsiveVector(cv::Mat &imgage )
//{

//}
