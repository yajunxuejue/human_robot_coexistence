#ifndef ROBOT_CONTROL_POSITION_H
#define ROBOT_CONTROL_POSITION_H
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <trac_ik/trac_ik.hpp>
#include "geometry_msgs/Pose.h"
#include <kdl_conversions/kdl_msg.h>

bool flag_calIK;
KDL::Chain chain;
KDL::JntArray nominal_, trac_ik_result;
KDL::Frame desired_tool0_pose;
KDL::JntArray ll, ul;//lower joint limits, upper joint limits
sensor_msgs::JointState joint_position_;
ros::Subscriber joint_states_sub_, comand_position_tool0_sub_;
ros::Publisher joint_position_com_pub_;

void iKcallback(const geometry_msgs::PointPtr tool0_com_pose);
void jointStateCallback(const sensor_msgs::JointState joint_state_);

#endif // ROBOT_CONTROL_POSITION_H
