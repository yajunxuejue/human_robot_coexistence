/*robot position contorl modle, subscribing two topics /joint_states and
 * kinect_merge/comand_position_tool0, the former topic comes from ur dirver,
 * the last one is advertised by closest_pt_tracking node in the kinect_huamn_tracking_ws packege*/
#include "pre_collision_free/robot_control_position.h"

void jointStateCallback(const sensor_msgs::JointState joint_state_)
  {

  for(uint i=0; i<6; i++){
      nominal_(i) = joint_state_.position[i];
    }
  flag_calIK=true;
  }

void iKcallback(geometry_msgs::Pose tool0_com_pose)
  {
    if(flag_calIK == true){
      TRAC_IK::TRAC_IK ik_solver("base", "tool0");
      if(!ik_solver.getKDLChain(chain)){
        ROS_ERROR("There was no valid KDL chain found");
        return ;
      }
      if(!ik_solver.getKDLLimits(ll, ul)){
        ROS_ERROR("There was no valid KDL joint limits found");
        return ;
      }
      assert(chain.getNrOfJoints() == ll.data.size());
      assert(chain.getNrOfJoints() == ul.data.size());
      tf::poseMsgToKDL(tool0_com_pose, desired_tool0_pose);
      int rc = ik_solver.CartToJnt(nominal_, desired_tool0_pose, trac_ik_result);
      if (rc >= 0){
         for(uint i=0; i<6; i++){
           joint_position_.position[i]= trac_ik_result(i);
        }
        ROS_INFO("J0=%f,J1=%f,J2=%f,J3=%f,J4=%f,J5=%f;",
                 trac_ik_result.data[0],trac_ik_result.data[1],trac_ik_result.data[2],
                 trac_ik_result.data[3],trac_ik_result.data[4],trac_ik_result.data[5]);
        joint_position_com_pub_.publish(joint_position_);
        }
      flag_calIK = false;
    }
  }

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "joint_position_comand");
  ros::NodeHandle nh;

  nominal_.resize(6);
  joint_position_.position.resize(6);

  joint_states_sub_=nh.subscribe("/joint_states", 10, jointStateCallback);
  comand_position_tool0_sub_=nh.subscribe<geometry_msgs::Pose>("kinect_merge/pose_comand", 1, iKcallback);
  joint_position_com_pub_=nh.advertise<sensor_msgs::JointState>("robot_joint_position", 2);

  ros::spin();
}
