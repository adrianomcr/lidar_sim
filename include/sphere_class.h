#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <vector>

#include <plane_constraint.h>

using namespace Eigen;

class sphere_class{
  private:

    double r, r2;
    VectorXd c;

    std::vector<plane_constraint *> plane_constraints;

  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    sphere_class(VectorXd, double); //Construtor
    sphere_class(double, double, double,    double); //Construtor
    ~sphere_class(); //Destructor



    void add_plane_constraint(double, double, double, double, double, double);
    double compute_distance(VectorXd, VectorXd);
    
};
