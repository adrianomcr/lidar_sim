#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
// #include <mutex>

using namespace Eigen;

class ellipsoid_class{
  private:

    double a,b,c,r012;
    VectorXd center, Omega_center;
    VectorXd r;
    MatrixXd Omega;


  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ellipsoid_class(double, double, double,    double, double, double); //Construtor
    ~ellipsoid_class(); //Destructor


    double compute_distance(VectorXd, VectorXd);


};
