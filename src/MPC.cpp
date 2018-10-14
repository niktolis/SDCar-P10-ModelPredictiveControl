#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
//#include "matplotlibcpp.h"

using CppAD::AD;

size_t N = 25;    /*!< Number/Length of timesteps */
double dt = 0.05; /*!< Duration of each timestep */

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Reference velocity
const double ref_v = 40.0;
// Reference Cross Track Error
const double ref_cte = 0.0;
// Refence Orientation Error
const double ref_epsi = 0.0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should establish
// where one variable starts and another ends.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  
  Eigen::VectorXd coeffs;               /*!< Fitted polynomial coefficients */
  const double rfc_cte_coeff = 1.0;     /*!< CTE reference state cost coefficient */
  const double rfc_epsi_coeff = 1.0;    /*!< Orientation error reference state cost coefficient */
  const double rfc_v_coeff = 1.0;       /*!< Velocity reference state cost coefficient */
  const double aus_delta_coeff = 1.0;   /*!< Steering angle actuation use cost coefficient */
  const double aus_a_coeff = 1.0;       /*!< Throttle actuation use cost coefficient */
  const double svgc_delta_coeff = 1.0;  /*!< Steering angle sequential value gap cost coeffiecient */
  const double svgc_a_coeff = 1.0;      /*!< Throttle value sequential gap cost coefficient */
  
  
  /**
   *  \brief Pass the coefficients to be evaluated
   *  \param coeffs The coefficient vector
   */
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  /**
   *  \brief Operator () overload
   *  \param fg is a vector containing the cost and constraints.
   *  \param vars is a vector containing the variable values (state & actuators).
   */
  void operator()(ADvector& fg, const ADvector& vars) {
    
    // The cost is stored in the first element of `fg`
    // Any additions to the cost should be added to `fg[0]`
    fg[0] = 0.0;
    
    // Reference State Cost
    // The part of the cost based on the reference state
    for (int t = 0; t < N; t++) {
      fg[0] += rfc_cte_coeff * CppAD::pow(vars[cte_start + t] - ref_cte , 2);
      fg[0] += rfc_epsi_coeff * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += rfc_v_coeff * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Actuation Use Cost
    // The part of the cost based on the use of the actuators (steering angle, throttle)
    for (int t = 0; t < N; t++) {
      fg[0] += aus_delta_coeff * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += aus_a_coeff * CppAD::pow(vars[a_start + t], 2);
    }
    
    // Sequential Value Gap Cost
    // The part of the cost based on the value gap between sequential actuations
    for (int t = 0; t < N - 2; t++) {
      fg[0] += svgc_delta_coeff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += svgc_a_coeff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    // Constraints
    
    // Initial Constraints
    //
    // We add 1 to each of the starting indices due to cost being located
    // at index 0 of `fg`
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      
      //The state at time t+1
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // The state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t -1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // The actuation is considered only at time t
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
      // Reference trajectory polynomial at time t
      AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      // Desired psi at time t
      AD<double> psides0 = CppAD::atan(coeffs[1]);
      
      // Model equations
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + (v[t] / Lf) * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * (delta[t] / Lf) * dt
      // The concept is to minimize following terms to reach the target
      // value which is 0
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set all non-actuators upper and lower limits
  // to the max negative and positive values
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = numeric_limits<double>::min();
    vars_upperbound[i] = numeric_limits<double>::max();
  }
  
  // The upper and lower limits of delata are set to -25 and 25
  // degrees. This shall be converted to radians.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = min_delta * 180.0 / M_PI;
    vars_upperbound[i] = max_delta * 180.0 / M_PI;
  }
  
  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = min_throttle;
    vars_upperbound[i] = max_throttle;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start]    = constraints_upperbound[x_start]    = x;
  constraints_lowerbound[y_start]    = constraints_upperbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = constraints_upperbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = constraints_upperbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = constraints_upperbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = epsi;
  
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {solution.x[x_start + 1], solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start], solution.x[a_start]};
}
