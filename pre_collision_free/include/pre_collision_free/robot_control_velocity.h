#ifndef ROBOT_CONTROL_VELOCITY_H
#define ROBOT_CONTROL_VELOCITY_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
//#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainiksolvervel_wdls.hpp>
//#include "../../../orocos_kinematics_dynamics/orocos_kdl/src/chainiksolvervel_wdls.hpp"
#include <trac_ik/trac_ik.hpp>
#include <trajectory_msgs/JointTrajectory.h>
//#include <kdl/solveri.hpp>

bool flag_calIK;
KDL::Chain chain;
KDL::JntArray joint_cur_position, ll, ul;

//KDL::SolverI carToJointVel;
trajectory_msgs::JointTrajectory joint_vel_com_;

ros::Subscriber velocity_com_sub, joint_states_sub;
ros::Publisher joint_velocity_com_pub;

void jointStateCallback(const sensor_msgs::JointState joint_state_);
void iKVelCallback(const geometry_msgs::Twist car_com_vel);

#endif //ROBOT_CONTROL_VELOCITY_H
