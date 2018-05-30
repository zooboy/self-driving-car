#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// the struct get_result is used for receiving the result from mpc solve() method.
struct get_result {

	vector<double> predlist_x;
	vector<double> predlist_y;
	double steer;
	double throttle;
};

// for receiving the coordinate transform result.
struct displaycoord {

	vector<double> x;
	vector<double> y;
};


class MPC {
 public:
  MPC();


  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  get_result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
