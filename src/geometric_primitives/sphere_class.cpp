#include "sphere_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


sphere_class::sphere_class(VectorXd c_0, double r_0){

  VectorXd c_init(3);
  c_init = c_0;
  c = c_init;

  r = r_0;
  r2 = r*r;
}

sphere_class::sphere_class(double cx, double cy, double cz, double r_0){

  VectorXd c_init(3);
  c_init << cx, cy, cz;
  c = c_init;

  r = r_0;
  r2 = r*r;
}


void sphere_class::add_plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  plane_constraints.push_back( new plane_constraint(nx,ny,nz,cx,cy,cz) );

}


double sphere_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position - c;
  v = direction;

  double gamma_1, gamma_2;
  double gamma_tmp;
  double B,C;

  B = 2*v.dot(x);
  C = x.dot(x)-r2;
  
  double delta = B*B-4*C;
  if(delta >= 0){
    double sqrt_delta = sqrt(delta);
    gamma_1 = (-B+sqrt_delta)/2.0; if (gamma_1<=0) {gamma_1 = 1e6;}
    gamma_2 = (-B-sqrt_delta)/2.0; if (gamma_2<=0) {gamma_2 = 1e6;}
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