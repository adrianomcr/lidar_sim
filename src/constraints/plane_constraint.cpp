#include "plane_constraint.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


plane_constraint::plane_constraint(VectorXd n_0, double d_0){

  VectorXd n_init(3);
  n_init = n_0;
  n = n_init;
  n = n/n.norm();

  d = d_0;

}

plane_constraint::plane_constraint(double nx, double ny, double nz, double d_0){

  VectorXd n_init(3);
  n_init << nx, ny, nz;
  n = n_init;
  n = n/n.norm();

  d = d_0;

}


plane_constraint::plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  VectorXd n_init(3), c_init(3);
  n_init << nx, ny, nz;
  c_init << cx, cy, cz;
  n = n_init;
  n = n/n.norm();
  d = n.dot(c_init);

}


double plane_constraint::check_validity(VectorXd position){

  VectorXd x(3);
  bool valid = false;

  x = position;

  if(x.dot(n) - d > 0){
    valid = true;
  }

  return valid;

}



// double plane_constraint::compute_distance(VectorXd position, VectorXd direction){

//   VectorXd x(3), v(3);
//   float gamma;
//   // for (int k=0; k<2 ; k++){

    
//     x = position;
//     v = direction;
//     // float gamma;

//     gamma = (d - x.dot(n))/v.dot(n);

//     if (gamma<=0){
//       gamma = 1e6;
//     }

//   // }
//   return gamma;

// }


