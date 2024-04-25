#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <vector>

#include <plane_constraint.h>


using namespace Eigen;

class ellipsoid_class{
  private:

    double a,b,c,r012;
    VectorXd center, Omega_center;
    VectorXd r;
    MatrixXd Omega;
    MatrixXd H_e_w, H_w_e, R_w_e, p_w_e;

    std::vector<plane_constraint *> plane_constraints;


  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ellipsoid_class(double, double, double,    double, double, double); //Construtor
    ellipsoid_class(double, double, double,    double, double, double,    double, double); //Construtor
    ~ellipsoid_class(); //Destructor

    void add_plane_constraint(double, double, double, double, double, double);
    double compute_distance(VectorXd, VectorXd);


};
