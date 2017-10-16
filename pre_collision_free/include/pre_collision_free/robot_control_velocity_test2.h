#ifndef ROBOT_CONTROL_VELOCITY_TEST2_H
#define ROBOT_CONTROL_VELOCITY_TEST2_H

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <moveit/move_group/node_name.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

static const std::string PLANNING_GROUP = "manipulator";

class UrVelControl
{
 private:
  Eigen::VectorXd qdot;
  trajectory_msgs::JointTrajectory joint_traj;

 public:
   UrVelControl(ros::NodeHandle &nh);
  ~UrVelControl();
  void ikVelSolverCallback(const geometry_msgs::Twist cmd_vel_car);

  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher  cmd_joint_speed_pub;
 // robot_model_loader::RobotModelLoader ur5_model_loader;
 // robot_model::RobotModel ur5_kinematic_model;
  robot_state::RobotStatePtr ur5_kinematic_state;
  moveit::planning_interface::MoveGroup move_group;
  const robot_state::JointModelGroup *joint_model_group;
};

#endif
