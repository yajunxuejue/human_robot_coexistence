#include "pre_collision_free/ur5_moveit.h"

UR5moveitcontrol::UR5moveitcontrol(ros::NodeHandle & nh)
  :nh_(nh)
  ,ur5_group("manipulator")
{
   target_pose.orientation.w = 1.0;
   target_pose.position.x = 0.3;
   target_pose.position.y = 0.3;
   target_pose.position.z = 0.3;
}

void UR5moveitcontrol::setTargetPose(geometry_msgs::Pose & target_pose_){
  target_pose = target_pose_;
}
