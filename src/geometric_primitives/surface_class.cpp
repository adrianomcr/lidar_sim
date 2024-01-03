#include "surface_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;



// constructor

surface_class::surface_class(VectorXd n_0, double d_0){

  VectorXd n_init(3);
  n_init = n_0;
  n = n_init;
  n = n/n.norm();

  d = d_0;

}

surface_class::surface_class(double nx, double ny, double nz, double d_0){

  VectorXd n_init(3);
  n_init << nx, ny, nz;
  n = n_init;
  n = n/n.norm();

  d = d_0;

}


double surface_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = position;
  v = direction;

  double gamma;

  gamma = (d - x.dot(n))/v.dot(n);

  if (gamma<=0){
    gamma = 1e6;
  }
  return gamma;

}


