# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Rubric

### The model

#### Kinematic vehicle model
This model is used to describe the kinematics of the vehicle. In this project,
this model is used to analyse the lateral and yaw vehicle kinematics. The main
states are the x-position, y-position, velocity and yaw-orientation. This states are
related to the center of mass of the planar vehicle model. The velocity is assumed
to be collinear to the vehicle's x-axis (pointing forward to the vehicle). Take it
into consideration, the update equations for the position and yaw orientation are obtained via

```
x(k+1) = x(k) + v(k) * cos(psi(k)) * dt;
y(k+1) = y(k) + v(k) * sin(psi(k)) * dt;
psi(k+1) = psi(k) + v(k) * delta(k) / Lf * dt;
```

#### Actuators
The actuator or control signals are the steering wheel angle and the throttle pedal.
The throttle pedal can be positive (for acceleration - full acceleration +1) or
negative (for braking - full braking -1). In this project, and for passenger cars,
we can limit the steering wheel angle to [-25 deg, +25 deg]. These are the variables
that the MPC will optimize and then apply to the vehicle kinematic model.


### Timestep Lenght & Elapsed duration
This values are the hyper-parameters of the MPC system developed for this project.
Theses hyper-parameters define the MPC horizon, i.e. how far in time the controller will
analyse and optimize in order to find the optimum value of the control's signals (steering wheel
angle and throttle pedal).

To tune these parameters, it is important to know the factible horizon for our MPC implementation.
This factible horizon depends mainly on the local power computation. In addition, a reasonable Elapsed duration should be proper chosen. To small values will be more computationally expensive and too large value
will decrese the MPC performance.

#### Trade-off
Taking into consideration the above, a value of N = 15 and DT = 0.1 was chosen and tested successfully.

### Polynomial Fitting and MPC Preprocessing
To obtain the polynomial coefficients, a set of points is required. At the beginning,
the vehicle's position, orientation and the waypoints are measured on the earth-fixed axis system of Unity.
In order to create a proper polynomial function, these set of points should be rotated to the
vehicle yaw orientation, this operation is done using the following transformation

```
  // rotation matrix from F to 0
  Eigen::MatrixXd A0F(2, 2);
  A0F << cos(-psi), -sin(-psi),
         sin(-psi), cos(-psi);

  // position vector for front vehicle point to polygonal points
  Eigen::VectorXd vec_pos0(2);  // in earth-fixed system

  // Need Eigen vectors for polyfit
  Eigen::VectorXd ptsx_car(ptsx.size());
  Eigen::VectorXd ptsy_car(ptsy.size());

  // rotate the position vector into the vehicle-fixed axis system
  for (size_t i = 0; i < ptsx.size(); i++){
    vec_pos0 << ptsx[i] - px, ptsy[i] - py;
    vec_pos0 = A0F*vec_pos0;
    ptsx_car[i] = vec_pos0(0);
    ptsy_car[i] = vec_pos0(1);
  }
```
Finally, the vectors ```ptsx``` and ```ptsy``` are employed to obtain the coefficients necessary to
compute the desired states values (cross-track error - CTE and yaw angle - EPSI).

### Model Predictive Control with Latency and Weights
At the beginning, with zero latency, the vehicle successfully make the lap with good perfomance.
If we add the required latency, i.e. 100ms, the vehicle also performed a good lap, but some time
a huge rate of the steering wheel angle is visualized. The weights for the cost function were calculated
taking into consideration the followings consideration:

* Small cross-track error (REF_CTE = 0.0)
* Small orientation error (REF_EPSI = 0.0)
* Good vehicle velocity tracking (REF_V = 60 mph)
* Smooth steering angle and acceleration transitions.

The weights used in this project were set as follows:

```
  // weights parameters for the cos function
  #define W_CTE 700.0
  #define W_EPSI 4000.0
  #define W_VEL 1.0
  #define W_DELTA 100000.0
  #define W_A 5.0
  #define W_DDELTA 20.0
  #define W_DA 10.0
  ```

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
