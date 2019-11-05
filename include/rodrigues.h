#include <Eigen/Dense>
#include <EigenTypes.h>

//Input:
//  omega - angular displacement vector
//Output: 
//  R - rotation matrix 
void rodrigues(Eigen::Matrix3d &R, Eigen::Ref<const Eigen::Vector3d> omega);