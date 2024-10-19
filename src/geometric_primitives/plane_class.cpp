#include "plane_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


plane_class::plane_class(double cx, double cy, double cz, double roll, double pitch, double yaw){

  VectorXd center_init(3), c_init(3);
  MatrixXd H_p_w_init(4,4), H_w_p_init(4,4);

  H_p_w_init << cos(pitch)*cos(yaw), -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw), cx,
                cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw), cy,
                -sin(pitch),         sin(roll)*cos(pitch),                                cos(roll)*cos(pitch),                               cz,
                0,                   0,                                                   0,                                                  1;
  H_w_p_init = H_p_w_init.inverse();

  n = H_p_w_init.block(0,2,3,1);
  n = n/n.norm();

  c_init << cx, cy, cz;
  center = c_init;

  d = n.dot(center);

  H_p_w = H_p_w_init;
  H_w_p = H_w_p_init;
  R_w_p = H_w_p_init.block(0,0,3,3);
  p_w_p = H_w_p_init.block(0,3,3,1);

}


void plane_class::add_plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  //  Transform the contraint (originaly written on the cylinder frame) to a frame aligned with the world but centered in the center of the cylinder
  VectorXd n_local(3), n_global(3), p_local(3), p_global(3);
  n_local << nx,ny,nz;
  n_global = R_w_p.transpose() * n_local;
  p_local << cx,cy,cz;
  p_global = R_w_p.transpose() * p_local + center;

  plane_constraints.push_back( new plane_constraint(n_global(0),n_global(1),n_global(2), p_global(0),p_global(1),p_global(2)) );

}


double plane_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  float gamma;

  x = position;
  v = direction;

  gamma = (d - x.dot(n))/v.dot(n);

  if (gamma<=0){
    return 1e6;
  }

  // Plane constraints
  Vector3d q;
  if(gamma < 1e6){
    q = x + v*gamma;
    for (int k=0; k<plane_constraints.size(); k++){
      if(!plane_constraints[k]->check_validity(q)){
        return 1e6;
      }
    }
      return gamma;
  }
  else{
    return 1e6;
  }

  std::cerr << "Caught exception: " << std::endl;
  return 1e6;
}
