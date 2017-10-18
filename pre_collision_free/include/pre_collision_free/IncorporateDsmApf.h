#ifndef INCORPORATEDSMAPF_H
#define INCORPORATEDSMAPF_H

#include <Eigen/Geometry>


class IncorporateDsmApf{
private:
  Eigen::Vector3f modulated_velocity;
  Eigen::Vector3f repulsive_vector;
  Eigen::Matrix3f modulation_matrix;
  bool direction_repvector_rawspeed; //direction between repulsive vector and raw command speed, true means the control point is approaching the obstacle surface.
  float min_distance;

  bool getDirection(Eigen::Vector3f &rep_vector, Eigen::Vector3f &raw_speed_);
  Eigen::Vector3f getModulatedVelocity() const;

public:
  IncorporateDsmApf(Eigen::Vector3f &repulsive_vector_, Eigen::Vector3f & raw_velocity_, float & min_distance_);
  void calculateModulatedVelocity(const Eigen::Vector3f &raw_speed_);
  void calculateModulationMatrix(Eigen::Vector3f & repulsive_vec);
  float calculateMagunitude(float & distance);
  ~IncorporateDsmApf();
};
#endif
