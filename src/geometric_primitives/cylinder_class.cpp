#include "cylinder_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


cylinder_class::cylinder_class(VectorXd n_0, VectorXd p_0, double r_0){

  VectorXd n_init(3), p_init(3);
  n_init = n_0;
  p_init = p_0;
  n = n_init;
  p = p_init;

  n = n/n.norm();

  r = r_0;
}


cylinder_class::cylinder_class(double nx, double ny, double nz, double px, double py, double pz, double r_0){

  VectorXd n_init(3), p_init(3);
  n_init << nx, ny, nz;
  p_init << px, py, pz;
  n = n_init;
  p = p_init;

  n = n/n.norm();

  r = r_0;

  lb = -1e6;
  ub = 1e6;
}


cylinder_class::cylinder_class(double nx, double ny, double nz, double px, double py, double pz, double r_0, double lb0, double ub0){

  VectorXd n_init(3), p_init(3);
  n_init << nx, ny, nz;
  p_init << px, py, pz;
  n = n_init;
  p = p_init;

  n = n/n.norm();

  r = r_0;

  lb = lb0;
  ub = ub0;
}


void cylinder_class::add_plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  plane_constraints.push_back( new plane_constraint(nx,ny,nz,cx,cy,cz) );

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

