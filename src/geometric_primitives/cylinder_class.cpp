#include "cylinder_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;



// constructor



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


  limit_function = &cylinder_class::nothing;

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


  limit_function = &cylinder_class::check_height;

}

double cylinder_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position;
  v = direction;

  double gamma, gamma_1, gamma_2;
  double A,B,C;

  Eigen::Vector3d Z;

  Z = (x-p)-(n.dot(x-p))*n;
  A = 1.0-(v.dot(n))*(v.dot(n));
  B = 2.0*v.dot(Z) - 2.0*(v.dot(n))*(n.dot(Z));
  C = Z.dot(Z)-r*r;
  if(B*B-4*A*C >= 0){
    gamma_1 = (-B+sqrt(B*B-4*A*C))/(2.0*A); if (gamma_1<=0) {gamma_1 = 1e6;}
    gamma_2 = (-B-sqrt(B*B-4*A*C))/(2.0*A); if (gamma_2<=0) {gamma_2 = 1e6;}
    gamma = gamma_1;
    if(gamma_2 < gamma_1){
      gamma = gamma_2;
    }
  }
  else{
    gamma = 1e6;
  }

  //cylinder_class::nothing(123);
  // (this->*limit_function)();

  if(n.dot(x+gamma*v - p)>ub){
    gamma = 1e6;
  }
  if(n.dot(x+gamma*v - p)<lb){
    gamma = 1e6;
  }

  return gamma;

}


void cylinder_class::nothing(){
  // cout << "nothing" << endl;
}


void cylinder_class::check_height(){
  // cout << "aaa" << endl;


  // gamma = 1e6;
}

// void set_cylinder_limits(double lb0, double ub0){
//   // lb = lb0;
//   // ub = ub0;
// }
