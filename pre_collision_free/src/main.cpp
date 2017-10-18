#include "pre_collision_free/ur5_moveit.h"
#include <Eigen/Geometry>
#include <iostream>

int main(int argc, char **argv)
{
  Eigen::Vector3f a, b, e;
  a<<1,2,3;
  b<<4,5,6;
  e<<7,8,9;
  Eigen::Matrix3f c;
  c << a,b,e;
  std::cout<<c;
  return 0;
}
