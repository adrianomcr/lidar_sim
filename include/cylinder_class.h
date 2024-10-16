#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <vector>

#include <plane_constraint.h>

using namespace Eigen;

class cylinder_class{
  private:

    double r;
    VectorXd n, p;
    double lb, ub;

    std::vector<plane_constraint *> plane_constraints;

  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    cylinder_class(VectorXd, VectorXd, double); //Construtor
    cylinder_class(double, double, double,    double, double, double,    double); //Construtor
    cylinder_class(double, double, double,    double, double, double,    double,    double, double); //Construtor
    ~cylinder_class(); //Destructor

    void add_plane_constraint(double, double, double, double, double, double);
    double compute_distance(VectorXd, VectorXd);
};
