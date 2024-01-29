#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
// #include <mutex>

using namespace Eigen;

class plane_class{
  private:

    double d;
    VectorXd n;


  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EKF_class(Eigen::VectorXd, double, double); //Construtor
    plane_class(VectorXd, double); //Construtor
    plane_class(double, double, double,    double); //Construtor
    plane_class(double, double, double,    double, double, double); //Construtor
    ~plane_class(); //Destructor


    double compute_distance(VectorXd, VectorXd);
    
};
