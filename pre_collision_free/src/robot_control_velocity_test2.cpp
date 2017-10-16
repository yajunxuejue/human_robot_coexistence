#include "pre_collision_free/robot_control_velocity_test2.h"



UrVelControl::UrVelControl(ros::NodeHandle &nh)
  :nh_(nh)
  {
    robot_model_loader::RobotModelLoader ur5_model_loader("robot_description");
    robot_model::RobotModelPtr ur5_kinematic_model = ur5_model_loader.getModel();
    ur5_kinematic_state(new robot_state::RobotState( ur5_kinematic_model));
    move_group("manipulator");
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO("Model frame: %s", ur5_kinematic_model->getModelFrame().c_str());
    cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>("/car_vel_com", 2, &UrVelControl::ikVelSolverCallback, this);
    cmd_joint_speed_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("cmd_joint_speed", 1);
  }

UrVelControl::~UrVelControl(){}

void UrVelControl::ikVelSolverCallback(const geometry_msgs::Twist cmd_vel_car)
  {
    Eigen::Matrix<double,6,1> cmd_vel_car_;
    tf::twistMsgToEigen(cmd_vel_car, cmd_vel_car_);
    ur5_kinematic_state->computeVariableVelocity(joint_model_group,
                                               qdot,
                                               cmd_vel_car_,
               ur5_kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()));
    joint_traj.points[0].velocities[0] = qdot(0);
    joint_traj.points[0].velocities[1] = qdot(1);
    joint_traj.points[0].velocities[2] = qdot(2);
    joint_traj.points[0].velocities[3] = qdot(3);
    joint_traj.points[0].velocities[4] = qdot(4);
    joint_traj.points[0].velocities[5] = qdot(5);
    cmd_joint_speed_pub.publish(joint_traj);
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_speed_control");
  ros::NodeHandle _nh;
  UrVelControl ur5control(_nh);
  ROS_INFO("robot speed control!!!");
  ros::spin();
}
