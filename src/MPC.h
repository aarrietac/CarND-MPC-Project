#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// MPC hyper-parameters
#define NUMBER_STEP 15
#define DT 0.1

// set reference values for the cost function
#define REF_CTE 0.0
#define REF_EPSI 0.0
#define REF_V 60

// weights parameters for the cos function
#define W_CTE 700.0
#define W_EPSI 4000.0
#define W_VEL 1.0
#define W_DELTA 100000.0
#define W_A 5.0
#define W_DDELTA 20.0
#define W_DA 10.0

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
