#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
// #include <mutex>

using namespace Eigen;

class sphere_class{
  private:

    double r;
    VectorXd c;

  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EKF_class(Eigen::VectorXd, double, double); //Construtor
    sphere_class(VectorXd, double); //Construtor
    sphere_class(double, double, double,    double); //Construtor
    ~sphere_class(); //Destructor


    double compute_distance(VectorXd, VectorXd);
    
};
