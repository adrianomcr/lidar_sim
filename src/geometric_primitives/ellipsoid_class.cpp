#include "ellipsoid_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;



// constructor

// ellipsoid_class::ellipsoid_class(VectorXd c_0, VectorXd r_0){

//   VectorXd center_init(3), r_init(3);
//   center_init = c_0;
//   r_init = r_0;
//   center = center_init;
//   r = r_init;

//   MatrixXd Omega_init(3,3);
//   Omega_init << r(1)*r(1)*r(2)*r(2), 0.0, 0.0,
//                 0.0, r(0)*r(0)*r(2)*r(2), 0.0,
//                 0.0, 0.0, r(0)*r(0)*r(1)*r(1);
//   Omega = Omega_init;
// }

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

  // Omega_diag << (1)*r(1)*r(2)*r(2),   r(0)*r(0)*r(2)*r(2),   r(0)*r(0)*r(1)*r(1);
  Omega_center = Omega*center;


  r012 = r(0)*r(0)*r(1)*r(1)*r(2)*r(2);
}


double ellipsoid_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position;
  v = direction;

  double gamma, gamma_1, gamma_2;
  double A, B, C;

  // cout << "\33[92m";
  // cout << v << endl << endl;
  // cout << "\33[93m";
  // cout << Omega << endl << endl;
  // cout << "\33[94m";
  // cout << Omega*v << endl << endl;
  // cout << "\33[95m";
  // cout << v.dot(Omega*v) << endl << endl;
  // cout << "---------" << endl;
  // cout << "\33[0m";



  // cout << v.dot(Omega*v) << endl;


  // A = v.dot(Omega*v);
  // B = 2.0*(x.dot(Omega*v) - v.dot(Omega*center));
  // C = x.dot(Omega*x) + center.dot(Omega*center) - 2.0*x.dot(Omega*center) - r(0)*r(0)*r(1)*r(1)*r(2)*r(2);
  
  Vector3d Omega_v;
  Omega_v << Omega*v;
  A = v.dot(Omega_v);
  B = 2.0*(x.dot(Omega_v) - v.dot(Omega_center));
  C = x.dot(Omega*x) + center.dot(Omega_center) - 2.0*x.dot(Omega_center) - r012;
  

  

  // A = 1;
  // B = 2.0*(x.dot(v) - v.dot(center));
  // C = x.dot(x) + center.dot(center) - 2.0*x.dot(center) - 1;

  // cout << Cx << " " << C << endl;

  double delta = B*B-4*A*C;
  if(delta >= 0){
    double sqrt_delta = sqrt(delta);
    gamma_1 = (-B+sqrt_delta)/(2.0*A); if (gamma_1<=0) {gamma_1 = 1e6;}
    gamma_2 = (-B-sqrt_delta)/(2.0*A); if (gamma_2<=0) {gamma_2 = 1e6;}
    // cout << gamma_1 << endl;
    // cout << gamma_1 << endl;
    gamma = gamma_1;
    if(gamma_2 < gamma_1){
      gamma = gamma_2;
    }
  }
  else{
    gamma = 1e6;
  }

  // cout << gamma << endl;
  return gamma;

  // return 1.0;
}



