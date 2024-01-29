#include "sphere_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;



// constructor

sphere_class::sphere_class(VectorXd c_0, double r_0){

  VectorXd c_init(3);
  c_init = c_0;
  c = c_init;

  r = r_0;

}

sphere_class::sphere_class(double cx, double cy, double cz, double r_0){

  VectorXd c_init(3);
  c_init << cx, cy, cz;
  c = c_init;

  r = r_0;

}


double sphere_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position;
  v = direction;

  double gamma, gamma_1, gamma_2;
  double B,C;

  B = 2*v.dot(x-c);
  C = (x-c).dot(x-c)-r*r;
  
  double delta = B*B-4*C;
  if(delta >= 0){
    double sqrt_delta = sqrt(delta);
    gamma_1 = (-B+sqrt_delta)/2.0; if (gamma_1<=0) {gamma_1 = 1e6;}
    gamma_2 = (-B-sqrt_delta)/2.0; if (gamma_2<=0) {gamma_2 = 1e6;}
    gamma = gamma_1;
    if(gamma_2 < gamma_1){
      gamma = gamma_2;
    }
  }
  else{
    gamma = 1e6;
  }
  
  return gamma;

}



