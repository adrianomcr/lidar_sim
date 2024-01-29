#include "plane_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


plane_class::plane_class(VectorXd n_0, double d_0){

  VectorXd n_init(3);
  n_init = n_0;
  n = n_init;
  n = n/n.norm();

  d = d_0;

}

plane_class::plane_class(double nx, double ny, double nz, double d_0){

  VectorXd n_init(3);
  n_init << nx, ny, nz;
  n = n_init;
  n = n/n.norm();

  d = d_0;

}


plane_class::plane_class(double nx, double ny, double nz, double cx, double cy, double cz){

  VectorXd n_init(3), c_init(3);
  n_init << nx, ny, nz;
  c_init << cx, cy, cz;
  n = n_init;
  n = n/n.norm();
  d = n.dot(c_init);

}


double plane_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position;
  v = direction;
  float gamma;

  gamma = (d - x.dot(n))/v.dot(n);

  if (gamma<=0){
    gamma = 1e6;
  }
  return gamma;

}


