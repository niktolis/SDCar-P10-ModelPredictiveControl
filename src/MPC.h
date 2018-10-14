#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
  
private:
  const double max_delta = 25.0;    /*!< Maximum steering angle in degrees */
  const double min_delta = -25.0;   /*!< Minimum steering angle in degrees */
  const double max_throttle = 1.0;  /*!< Maximum throttle */
  const double min_throttle = -1.0; /*!< Minimum throttle */
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
