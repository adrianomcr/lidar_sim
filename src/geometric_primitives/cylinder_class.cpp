#include "cylinder_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


cylinder_class::cylinder_class(double cx, double cy, double cz, double roll, double pitch, double yaw, double r_0){

  VectorXd center_init(3), n_init(3);
  MatrixXd H_c_w_init(4,4), H_w_c_init(4,4);

  center_init << cx, cy, cz;
  r = r_0;
  center = center_init;
  p = center_init;
  
  H_c_w_init << cos(pitch)*cos(yaw), -cos(roll)*sin(yaw) + sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw), cx,
              cos(pitch)*sin(yaw), cos(roll)*cos(yaw) + sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw) + cos(roll)*sin(pitch)*sin(yaw), cy,
              -sin(pitch),         sin(roll)*cos(pitch),                                cos(roll)*cos(pitch),                               cz,
              0,                   0,                                                   0,                                                  1;
  H_w_c_init = H_c_w_init.inverse();

  n_init = H_c_w_init.block(0,2,3,1);
  n = n_init;
  n = n/n.norm();
  
  H_c_w = H_c_w_init;
  H_w_c = H_w_c_init;
  R_w_c = H_w_c_init.block(0,0,3,3);
  p_w_c = H_w_c_init.block(0,3,3,1);
}



void cylinder_class::add_plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  //  Transform the contraint (originaly written on the cylinder frame) to a frame aligned with the world but centered in the center of the cylinder
  VectorXd n_local(3), n_global(3), p_local(3), p_global(3);
  n_local << nx,ny,nz;
  n_global = R_w_c.transpose() * n_local;
  p_local << cx,cy,cz;
  p_global = R_w_c.transpose() * p_local;

  plane_constraints.push_back( new plane_constraint(n_global(0),n_global(1),n_global(2), p_global(0),p_global(1),p_global(2)) );

  //  plane_constraints.push_back( new plane_constraint(nx,ny,nz,cx,cy,cz) );

}


double cylinder_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position-p;
  v = direction;

  double gamma_1, gamma_2;
  double gamma_tmp;
  double A,B,C;

  Eigen::Vector3d Z;
  float vn = v.dot(n);

  Z = x-(n.dot(x))*n;
  A = 1.0-vn*vn;
  B = 2.0*(v.dot(Z) - vn*(n.dot(Z)));
  C = Z.dot(Z)-r*r;

  double delta = B*B-4*A*C;
  if(delta >= 0){
    double sqrt_delta = sqrt(delta);
    gamma_1 = (-B+sqrt_delta)/(2.0*A); if (gamma_1<=0) {gamma_1 = 1e6;}
    gamma_2 = (-B-sqrt_delta)/(2.0*A); if (gamma_2<=0) {gamma_2 = 1e6;}
    // Force gamma_1 < gamma_2
    if(gamma_2 < gamma_1){
      gamma_tmp = gamma_1;
      gamma_1 = gamma_2;
      gamma_2 = gamma_tmp;
    }
  }
  else{
    return 1e6;
  }


  // Plane constraints
  Vector3d q;
  if(gamma_1 < 1e6){
    q = x + v*gamma_1;
    for (int k=0; k<plane_constraints.size(); k++){
      if(!plane_constraints[k]->check_validity(q)){
        gamma_1 = 1e6;
        break;
      }
    }
    if(gamma_1 < 1e6){
      return gamma_1;
    }
    else{
      if(gamma_2 < 1e6){
        q = x + v*gamma_2;
        for (int k=0; k<plane_constraints.size(); k++){
          if(!plane_constraints[k]->check_validity(q)){
            return 1e6;
          }
        }
        return gamma_2;
      }
      else{
        return 1e6;
      }
    }
  }
  else{
    return 1e6;
  }

  std::cerr << "Caught exception: " << std::endl;
  return 1e6;
}

