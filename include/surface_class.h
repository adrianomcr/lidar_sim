#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
// #include <mutex>

using namespace Eigen;

class surface_class{
  private:

    double d;
    VectorXd n;


  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EKF_class(Eigen::VectorXd, double, double); //Construtor
    surface_class(VectorXd, double); //Construtor
    surface_class(double, double, double,    double); //Construtor
    ~surface_class(); //Destructor


    double compute_distance(VectorXd, VectorXd);
    
};
