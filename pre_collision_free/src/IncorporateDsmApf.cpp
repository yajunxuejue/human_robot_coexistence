#include "pre_collision_free/IncorporateDsmApf.h"

//public functions
IncorporateDsmApf::IncorporateDsmApf(Eigen::Vector3f & repulsive_vector_, Eigen::Vector3f &raw_velocity_, float & min_distance_)
{
  replulsive_vector = repulsive_vector_;
  min_distance = min_distance_;
  raw_velocity = raw_velocity_;
  direction_repvector_rawspeed = getDirection(repulsive_vector_, raw_velocity_);
  calculateModulationMatrix(repulsive_vector_);
}

float IcorporateDsmApf::calculateMagunitude(float &distance)
{
  return (1.0 + var)/(1+exp(5*distance-1)*8);
}

void IncorporateDsmApf::calculateModulationMatrix(Eigen::Vector3f &repulsive_vec)
{
  float repulsive_vec_norm = repulsive_vec.norm();
  Eigen::Matrix3f diagonal_matrix;
  Eigen::Vector3f unit1_repulsive_vector;
  Eigen::Vector3f unit2_repulsive_vector;
  Eigen::Vector3f unit3_repulsive_vector;
  unit1_repulsive_vector = repulsive_vec/repulsive_vec_norm;
  unit2_repulsive_vector << unit1_repulsive_vector(1), -unit1_repulsive_vector(0), 0;
  unit3_repulsive_vector = unit1_repulsive_vector.cross(unit2_repulsive_vector);
  modulation_matrix << unit1_repulsive_vector, unit2_repulsive_vector,unit3_repulsive_vector;
  float magunitude = calculateMagunitude(min_distance);
  if(direction_repvector_rawspeed){
    diagonal_matrix<<1 - 2*magunitude,    0,             0,
                             0,      1 + magunitude,     0,
                             0,           0,       1 + magunitude;
    modulation_matrix = modulation_matrix.transpose() * diagonal_matrix * modulation_matrix;
  }else{
    diagonal_matrix<<1 + magunitude,    0,             0,
                             0,    1 + magunitude,     0,
                             0,         0,       1 + magunitude;
    modulation_matrix = modulation_matrix.transpose() * diagonal_matrix * modulation_matrix;
  }
}

void IncorporateDsmApf::calculateModulatedVelocity(Eigen::Vector3f & modulated_velocity_out)
{
  modulated_velocity_out = modulation_matrix * raw_velocity;
}

IncorporateDsmApf::~IncorporateDsmApf()
{

}

// private functions
bool IncorporateDsmApf::getDirection(Eigen::Vector3f &rep_vector, Eigen::Vector3f &raw_speed_)
{
  if(rep_vector * raw_speed_ < 0){
   return true;
  }
  else{
   return false;
  }
}
