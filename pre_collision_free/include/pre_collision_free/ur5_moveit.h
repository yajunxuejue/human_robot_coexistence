#ifndef UR5_MOVEIT_H
#define UR5_MOVEIT_H
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <string>

class UR5moveitcontrol{
private:
  ros::NodeHandle nh_;
  moveit::planning_interface::MoveGroup ur5_group;
  geometry_msgs::Pose target_pose;

public:
  UR5moveitcontrol(ros::NodeHandle & nh);
  void setTargetPose(geometry_msgs::Pose & target_pose_);
  ~UR5moveitcontrol(){}
};
#endif
