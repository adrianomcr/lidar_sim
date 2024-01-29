#include "ellipsoid_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


ellipsoid_class::ellipsoid_class(double a_0, double b_0, double c_0, double cx, double cy, double cz){

  VectorXd center_init(3), r_init(3);
  center_init << cx, cy, cz;
  r_init << a_0, b_0, c_0;
  center = center_init;
  r = r_init;

  MatrixXd Omega_init(3,3);
  Omega_init << r(1)*r(1)*r(2)*r(2), 0.0, 0.0,
                0.0, r(0)*r(0)*r(2)*r(2), 0.0,
                0.0, 0.0, r(0)*r(0)*r(1)*r(1);
  Omega = Omega_init;

  Omega_center = Omega*center;


  r012 = r(0)*r(0)*r(1)*r(1)*r(2)*r(2);
}


double ellipsoid_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position;
  v = direction;

  double gamma, gamma_1, gamma_2;
  double A, B, C;


  // TODO: Consider here ellipsoids with rotations as well

  
  Vector3d Omega_v;
  Omega_v << Omega*v;
  A = v.dot(Omega_v);
  B = 2.0*(x.dot(Omega_v) - v.dot(Omega_center));
  C = x.dot(Omega*x) + center.dot(Omega_center) - 2.0*x.dot(Omega_center) - r012;

  double delta = B*B-4*A*C;
  if(delta >= 0){
    double sqrt_delta = sqrt(delta);
    gamma_1 = (-B+sqrt_delta)/(2.0*A); if (gamma_1<=0) {gamma_1 = 1e6;}
    gamma_2 = (-B-sqrt_delta)/(2.0*A); if (gamma_2<=0) {gamma_2 = 1e6;}
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



