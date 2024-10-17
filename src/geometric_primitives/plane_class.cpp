#include "plane_class.h"

#define PI 3.1415926535

using namespace std;
using namespace Eigen;


// plane_class::plane_class(VectorXd n_0, double d_0){

//   VectorXd n_init(3);
//   n_init = n_0;
//   n = n_init;
//   n = n/n.norm();

//   d = d_0;
// }

// plane_class::plane_class(double nx, double ny, double nz, double d_0){

//   VectorXd n_init(3);
//   n_init << nx, ny, nz;
//   n = n_init;
//   n = n/n.norm();

//   d = d_0;
// }


plane_class::plane_class(double nx, double ny, double nz, double cx, double cy, double cz){

  VectorXd n_init(3), c_init(3);
  n_init << nx, ny, nz;
  c_init << cx, cy, cz;
  center = c_init;
  n = n_init;
  n = n/n.norm();
  d = n.dot(c_init);
}


void plane_class::add_plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  plane_constraints.push_back( new plane_constraint(nx,ny,nz,cx,cy,cz) );

}


double plane_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  float gamma;

  x = position;
  v = direction;

  gamma = (d - x.dot(n))/v.dot(n);

  if (gamma<=0){
    return 1e6;
  }

  // Plane constraints
  Vector3d q;
  if(gamma < 1e6){
    q = x-center + v*gamma;
    for (int k=0; k<plane_constraints.size(); k++){
      if(!plane_constraints[k]->check_validity(q)){
        return 1e6;
      }
    }
      return gamma;
  }
  else{
    return 1e6;
  }

  std::cerr << "Caught exception: " << std::endl;
  return 1e6;
}
