#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <memory>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/*
Abstract Pure Virtual class to help with uit test and mocking
*/
class ITools {
public:
  ITools(){};
  virtual ~ITools(){};
  virtual VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) = 0;
  virtual MatrixXd CalculateJacobian(const VectorXd& x_state) = 0;
  virtual VectorXd p2c(const VectorXd& polar) = 0;
  virtual VectorXd c2p(const VectorXd& cartesian) = 0;
};

class Tools: public ITools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /*
  * A helper method to convert polar to cartesian
  */

  VectorXd p2c(const VectorXd& polar);

  /*
  * A helper method to map cartesian to polar 
  */

  VectorXd c2p(const VectorXd& cartesian);

};

#endif /* TOOLS_H_ */
