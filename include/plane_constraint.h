#ifndef PLANE_CONSTRAINT_H_
#define PLANE_CONSTRAINT_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>

using namespace Eigen;

class plane_constraint{
  private:

    double d;
    VectorXd n;


  public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EKF_class(Eigen::VectorXd, double, double); //Construtor
    plane_constraint(VectorXd, double); //Construtor
    plane_constraint(double, double, double,    double); //Construtor
    plane_constraint(double, double, double,    double, double, double); //Construtor
    ~plane_constraint(); //Destructor


    double check_validity(VectorXd);   
};

#endif