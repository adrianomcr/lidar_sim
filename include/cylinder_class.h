#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
// #include <mutex>

using namespace Eigen;

class cylinder_class{
  private:

    double r;
    VectorXd n, p;

    double lb, ub;

    void (cylinder_class::*limit_function)();


  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EKF_class(Eigen::VectorXd, double, double); //Construtor
    cylinder_class(VectorXd, VectorXd, double); //Construtor
    cylinder_class(double, double, double,    double, double, double,    double); //Construtor
    cylinder_class(double, double, double,    double, double, double,    double,    double, double); //Construtor
    ~cylinder_class(); //Destructor


    double compute_distance(VectorXd, VectorXd);

    void nothing();

    void check_height();


    // void set_cylinder_limits(double, double);
    

};
