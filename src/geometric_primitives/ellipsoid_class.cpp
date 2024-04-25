#include "ellipsoid_class.h"


#define PI 3.1415926535

using namespace std;
using namespace Eigen;


ellipsoid_class::ellipsoid_class(double a_0, double b_0, double c_0, double cx, double cy, double cz){

  VectorXd center_init(3), r_init(3);
  MatrixXd H_e_w_init(4,4), H_w_e_init(4,4);

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

  double pitch = 0;
  double yaw = 0;
  H_e_w_init << cos(pitch)*cos(yaw), -sin(yaw), cos(yaw)*sin(pitch), cx,
                cos(pitch)*sin(yaw), cos(yaw) , sin(pitch)*sin(yaw), cy,
                -sin(pitch)        , 0        , cos(pitch)         , cz,
                0                  , 0        , 0                  , 1;
  H_w_e_init = H_e_w_init.inverse();

  H_e_w = H_e_w_init;
  H_w_e = H_w_e_init;
  R_w_e = H_w_e_init.block(0,0,3,3);
  p_w_e = H_w_e_init.block(0,3,3,1);

}


ellipsoid_class::ellipsoid_class(double a_0, double b_0, double c_0, double cx, double cy, double cz, double pitch, double yaw){

  VectorXd center_init(3), r_init(3);
  MatrixXd H_e_w_init(4,4), H_w_e_init(4,4);

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


  H_e_w_init << cos(pitch)*cos(yaw), -sin(yaw), cos(yaw)*sin(pitch), cx,
                cos(pitch)*sin(yaw), cos(yaw) , sin(pitch)*sin(yaw), cy,
                -sin(pitch)        , 0        , cos(pitch)         , cz,
                0                  , 0        , 0                  , 1;
  H_w_e_init = H_e_w_init.inverse();

  H_e_w = H_e_w_init;
  H_w_e = H_w_e_init;
  R_w_e = H_w_e_init.block(0,0,3,3);
  p_w_e = H_w_e_init.block(0,3,3,1);

}



void ellipsoid_class::add_plane_constraint(double nx, double ny, double nz, double cx, double cy, double cz){

  plane_constraints.push_back( new plane_constraint(nx,ny,nz,cx,cy,cz) );

}


double ellipsoid_class::compute_distance(VectorXd position, VectorXd direction){

  VectorXd x(3), v(3);
  x = R_w_e*position + p_w_e;
  v = R_w_e*direction;  

  double gamma, gamma_1, gamma_2;
  double A, B, C;

  Vector3d Omega_v;
  Omega_v << Omega(0,0)*v(0),Omega(1,1)*v(1),Omega(2,2)*v(2);
  A = v.dot(Omega_v);
  B = 2.0*x.dot(Omega_v);
  C = Omega(0,0)*x(0)*x(0)+Omega(1,1)*x(1)*x(1)+Omega(2,2)*x(2)*x(2) - r012;

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


  if(gamma<1e6){
    Vector3d q;
    q = x+v*gamma;
    for (int k=0; k<plane_constraints.size(); k++){
      if(!plane_constraints[k]->check_validity(q)){
        gamma = 1e6;
        break;
      }
    }
  }

  return gamma;

}



