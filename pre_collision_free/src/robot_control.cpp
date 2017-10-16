#include <ros/ros.h>
#include "pre_collision_free/robot_control.h"
#include <kdl/chainiksolverpos_nr_jl.hpp>

static const std::string OPENCV_WINDOW = "Image window";
//static const std::string PLANNING_GROUP = "ur5";

RepulsiveVector:: RepulsiveVector(ros::NodeHandle &nh)
    :it_(nh)
    ,nh_(nh)
    ,region_surveillance_height(100)
    ,region_surveillance_width (100)
    ,flag_calIK(false)
  {
    nominal.resize(6);
    cam_rgb_tf_base_rot = Eigen::AngleAxisf( 1.173, Eigen::Vector3f::UnitZ())
                        * Eigen::AngleAxisf( 0.000, Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(-2.063, Eigen::Vector3f::UnitX());
    joint_states_sub_=nh_.subscribe("/joint_states", 1000, &RepulsiveVector::jointStateCallback, this);
    image_sub_ = it_.subscribe("/camera/depth_registered_filtered/image_rect", 10, &RepulsiveVector::repulsiveVectorCallback,this);

    joint_position_pub = nh_.advertise<sensor_msgs::JointState>("robot_joint_position", 1000);

    cv::namedWindow(OPENCV_WINDOW);
  }
RepulsiveVector::~RepulsiveVector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
void RepulsiveVector::repulsiveVectorCallback(const sensor_msgs::ImageConstPtr& flitered_depth_image)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
      if(flitered_depth_image -> encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
         cv_ptr = cv_bridge::toCvShare(flitered_depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
         cv_flitered_depth_image = cv_ptr -> image;
      }
      else
      {
        cv_ptr = cv_bridge::toCvShare(flitered_depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_ptr -> image.convertTo(cv_flitered_depth_image, CV_32F, 0.001);
      }

    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cv::Mat image_ROI =  cv_flitered_depth_image(cv::Rect(control_point_x + 10, control_point_y + 10, 20, 20));
    getControlPoints();
    calculateRepulsiveVector();
    //int c=cv_flitered_depth_image.channels();
    //float c=cvGetReal2D(&cv_flitered_depth_image, 464,196);
    //float a=cv_flitered_depth_image.at<float>(tool0_control_point_y, tool0_control_point_x);//NOTE:at< >(row,col)
    //std::cout<<a<<std::endl;
    cv::circle(cv_flitered_depth_image, cv::Point(tool0_control_point_x, tool0_control_point_y),10, CV_RGB(255,255,255), 3);
    //cv::circle(cv_flitered_depth_image, cv::Point(100, 100), 10, CV_RGB(255,255,255), 3);
    cv::imshow(OPENCV_WINDOW, cv_flitered_depth_image);
    cv::waitKey(3);

  }

void RepulsiveVector::getControlPoints()
  {
    //get control points of robot and mapping them into image frame, just using tf listener.
    tf::StampedTransform tool0_to_camera_transform;
    try {
      listener_.waitForTransform("camera_rgb_optical_frame", "tool0", ros::Time (0), ros::Duration(3.0));
      listener_.lookupTransform ("camera_rgb_optical_frame", "tool0", ros::Time (0), tool0_to_camera_transform); //reference frame first, than target frame.
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }
     //mapping the control points into image frame
     tool0_control_point_x  = 525*(tool0_to_camera_transform.getOrigin().x())/tool0_to_camera_transform.getOrigin().z() + 319.5;
     tool0_control_point_y  = 525*(tool0_to_camera_transform.getOrigin().y())/tool0_to_camera_transform.getOrigin().z() + 239.5;
     tool0_control_point_dz = tool0_to_camera_transform.getOrigin().z();
     //std::cout<<tool0_control_point_x<<","<<tool0_control_point_y<<","<<tool0_control_point_dz<<std::endl;
  }

void RepulsiveVector::calculateRepulsiveVector()
  {
    int obstacle_x, obstacle_y;
    float obstacle_dz, minDis_ctrolpoint_obs = 1000;
    Eigen::Vector3f repulsive_vector, sum_repulsive_vector;
    KDL::Rotation com_tool0_rot = KDL::Rotation::RPY(-3.125, -0.209, -1.397);
    KDL::Vector com_position_tool0;
    sum_repulsive_vector << 0,0,0;
      for(uint i=0; i<region_surveillance_width; i=i+3)
      {
        for(uint j=0; j<region_surveillance_height; j=j+3)
        {
           obstacle_x = tool0_control_point_x - region_surveillance_width/2 + i;
           obstacle_y = tool0_control_point_y - region_surveillance_height/2 + j;
           obstacle_dz = cv_flitered_depth_image.at<float>(obstacle_y, obstacle_x);

           if( fabs(obstacle_dz - tool0_control_point_dz) < 0.5 && fabs(obstacle_dz - tool0_control_point_dz) > 0.1)
           {
             repulsive_vector(0,0) = ((obstacle_x - 525)*obstacle_dz - (tool0_control_point_x-525)*tool0_control_point_dz)/319.5;
             repulsive_vector(1,0) = ((obstacle_y - 525)*obstacle_dz - (tool0_control_point_y-525)*tool0_control_point_dz)/239.5;
             repulsive_vector(2,0) = obstacle_dz - tool0_control_point_dz;
             std::cout<<repulsive_vector.norm()<<std::endl;
             sum_repulsive_vector = repulsive_vector + sum_repulsive_vector;
             //std::cout<<repulsive_vector<<std::endl;
             if(repulsive_vector.norm() < minDis_ctrolpoint_obs){
                minDis_ctrolpoint_obs = repulsive_vector.norm();
             }
           }
        }
      }
    repulsive_vector = sum_repulsive_vector / 1089;
    repulsive_vector =  cam_rgb_tf_base_rot
                      * (0.002/(1 + exp((minDis_ctrolpoint_obs*0.02 - 1)*5)))
                      * (repulsive_vector / repulsive_vector.norm());
    tf::StampedTransform tool0_to_base_transform;
    try {
      listener_.waitForTransform("base", "tool0", ros::Time (0), ros::Duration(3.0));
      listener_.lookupTransform ("base", "tool0", ros::Time (0), tool0_to_base_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return;
    }
    com_position_tool0[0] = repulsive_vector(0,0)+tool0_to_base_transform.getOrigin().x();
    com_position_tool0[1] = repulsive_vector(1,0)+tool0_to_base_transform.getOrigin().y();
    com_position_tool0[2] = repulsive_vector(2,0)+tool0_to_base_transform.getOrigin().z();
    KDL::Frame com_tool0_pose(com_tool0_rot, com_position_tool0);
    calculateIK(com_tool0_pose);
  }

void RepulsiveVector::jointStateCallback(const sensor_msgs::JointState _jointstate)
  {
//    for(uint i=0; i<6; i++){
//      std::cout<<_jointstate.position[i]<<std::endl;
//    }
    //std::cout<<_jointstate.position[0];
    for(uint i=0; i<6; i++){
        nominal(i) = _jointstate.position[i];
      }
    flag_calIK=true;
  }

void RepulsiveVector::calculateIK(KDL::Frame desired_end_effector_pose)
  {
    if(flag_calIK == true){
    TRAC_IK::TRAC_IK ik_solver("base", "tool0");
    KDL::Chain chain;
    KDL::JntArray result_trac_ik, nominal(6);
    sensor_msgs::JointState robot_joint_position;
    robot_joint_position.position.resize(6);
    if(!ik_solver.getKDLChain(chain)){
      ROS_ERROR("There was no valid KDL chain found");
      return;
    }
    if(!ik_solver.getKDLLimits(ll, ul)){
      ROS_ERROR("There was no valid KDL joint limits found");
      return;
    }
    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());
   // nominal(chain.getNrOfJoints());
//        nominal.data[0]=1.74442;
//        nominal.data[1]=-1.92287;
//        nominal.data[2]=1.12897;
//        nominal.data[3]=-0.985589;
//        nominal.data[4]=-1.55417;
//        nominal.data[5]=-1.57037;
    //tf::poseMsgToKDL(pose_c,end_effector_pose);
    int rc = ik_solver.CartToJnt(nominal, desired_end_effector_pose, result_trac_ik);
    if (rc >= 0){
       for(uint i=0; i<6; i++){
         robot_joint_position.position[i]= result_trac_ik(i);
      }
      ROS_INFO("J0=%f,J1=%f,J2=%f,J3=%f,J4=%f,J5=%f;",
               result_trac_ik.data[0],result_trac_ik.data[1],result_trac_ik.data[2],
               result_trac_ik.data[3],result_trac_ik.data[4],result_trac_ik.data[5]);
      joint_position_pub.publish(robot_joint_position);
      }
    }
  }

void RepulsiveVector::test_trac_ik()
  {
    KDL::Frame cam_optTobase;
    KDL::Rotation tool0_base_rot;
    KDL::Vector tool0_position_next;
    //cam_optTobase = (tool0_base_rot, tool0_position_next);


    tf::Pose com_pose;
    tf::Vector3 com_pose_position;
    com_pose_position.setValue(0,0,0);
    tf::Quaternion com_pose_orien;
    com_pose_orien.setValue(0,0,0,0);
    com_pose.setOrigin(com_pose_position);
    com_pose.setRotation(com_pose_orien);

  }
//void RepulsiveVector::robotControl(Eigen::Vector3f sum_repulsive_vector_)
//  {
//  moveit::planning_interface::MoveGroup move_group("PLANNING_GROUP");
//  ROS_INFO("Reference frame: %s", group.getplanningFrame().c_str());
//  //set the target pose of
//  geometry_msgs::Pose target_pose;
//  target_pose.orientation.x = 0.0;
//  target_pose.orientation.y = 0.0;
//  target_pose.orientation.z = 0.0;
//  target_pose.orientation.w = 0.0;
//  target_pose.position.x = 0.0;
//  target_pose.position.y = 0.0;
//  target_pose.position.z = 0.0;
//  group.setPoseTarget(target_pose);

//  //starting motion planning
//  moveit::planning_interface::MoveGroup::Plan ur5_plan;
//  bool success = group.plan(ur5_plan);
//  //ROS_INFO();
//  if (success)
//    {
//      group.execute(ur5_plan);
//    }
//  }


int main(int argc, char **argv)
{
  // Set up ROS.
  //ros::init(argc, argv, "robot_control");
  //ros::NodeHandle nh;
  //RepulsiveVector r(nh);
  //ROS_INFO("Hello world!");
  //ros::spin();
  cv::VideoCapture cap("videotestsrc ! videoconvert ! appsink");
  if (!cap.isOpened()) {
      printf("=ERR= can't create video capture\n");
      return -1;
  }
}
