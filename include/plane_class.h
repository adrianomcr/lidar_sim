#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <vector>

#include <plane_constraint.h>

using namespace Eigen;

class plane_class{
  private:

    double d;
    VectorXd n;
    VectorXd center;

    std::vector<plane_constraint *> plane_constraints;

  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // plane_class(VectorXd, double); //Construtor
    // plane_class(double, double, double,    double); //Construtor
    plane_class(double, double, double,    double, double, double); //Construtor
    ~plane_class(); //Destructor

    void add_plane_constraint(double, double, double, double, double, double);
    double compute_distance(VectorXd, VectorXd);
};
