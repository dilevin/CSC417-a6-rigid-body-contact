#include <Eigen/Dense>
#include <EigenTypes.h>

//Input:
//  x - world space position
//  R - rotation from undeformed to world space
//  p - center-of-mass translation 
//Output:
//  X - undeformed position of point x
//  qdot - updated generalized velocities 
void inverse_rigid_body(Eigen::Vector3d &X, Eigen::Ref<const Eigen::Vector3d> x, 
                        Eigen::Ref<const Eigen::Matrix3d> R, Eigen::Ref<const Eigen::Vector3d> p);