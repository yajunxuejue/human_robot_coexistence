#include "pre_collision_free/robot_control_velocity.h"

void jointStateCallback(const sensor_msgs::JointState joint_state_)
  {
  for(uint i=0; i<joint_state_.position.size(); i++){
      joint_cur_position.data[i] = joint_state_.position[i];
      //std::cout<<joint_cur_position.data<<std::endl;
    }
  flag_calIK = true;
  }
void iKVelCallback(const geometry_msgs::Twist car_com_vel)
  {
    if(flag_calIK){
    KDL::Twist car_com_vel_;
    tf::twistMsgToKDL(car_com_vel, car_com_vel_);
    KDL::JntArray q(chain.getNrOfJoints()), qdot(chain.getNrOfJoints());
    KDL::JntArrayVel joint_vel_com(q, qdot);
    int rc;
    KDL::FrameVel _car_com_vel;
    _car_com_vel.p.v.data[0] = car_com_vel_.vel.data[0];
    _car_com_vel.p.v.data[1] = car_com_vel_.vel.data[1];
    _car_com_vel.p.v.data[2] = car_com_vel_.vel.data[2];
    _car_com_vel.M.w.data[0] = 0;
    _car_com_vel.M.w.data[1] = 0;
    _car_com_vel.M.w.data[2] = 0;
    std::cout<<_car_com_vel.deriv().vel.data<<std::endl;
    KDL::ChainIkSolverVel_wdls ik_solver_vel(chain);
    rc = ik_solver_vel.CartToJnt(joint_cur_position, _car_com_vel, joint_vel_com);
    if(rc < 0){
      return;
      }else{
        joint_vel_com_.points[0].velocities[0] = joint_vel_com.qdot(0);
        joint_vel_com_.points[0].velocities[1] = joint_vel_com.qdot(1);
        joint_vel_com_.points[0].velocities[2] = joint_vel_com.qdot(2);
        joint_vel_com_.points[0].velocities[3] = joint_vel_com.qdot(3);
        joint_vel_com_.points[0].velocities[4] = joint_vel_com.qdot(4);
        joint_vel_com_.points[0].velocities[5] = joint_vel_com.qdot(5);
        joint_velocity_com_pub.publish(joint_vel_com_);
        flag_calIK = false;
      }
    }
  }

int main(int argc, char **argv)
{
  flag_calIK = false;
  joint_cur_position.resize(6);
  ros::init(argc,argv,"joint_velocity_control");
  ros::NodeHandle nh;
  TRAC_IK::TRAC_IK ik_solver("base", "tool0");
  if(!ik_solver.getKDLChain(chain)){
    ROS_ERROR("There was no valid KDL chain found");
    return 0;
  }
  if(!ik_solver.getKDLLimits(ll, ul)){
    ROS_ERROR("There was no valid KDL joint limits found");
    return 0;
  }
  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());


  joint_states_sub = nh.subscribe("/joint_states", 10, jointStateCallback);
  velocity_com_sub = nh.subscribe<geometry_msgs::Twist>("/car_vel_com", 1, iKVelCallback);
  joint_velocity_com_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/cmd_joint_speed", 1);
  ROS_INFO("robot velocity control!!!");
  ros::spin();
}
