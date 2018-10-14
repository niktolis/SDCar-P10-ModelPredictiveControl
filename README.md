# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview

The project contains the implementation of a Model Predictive Controller ([MPC](https://en.wikipedia.org/wiki/Model_predictive_control)) for the Udacity Self Driving Car Nanodegree program. The target of the controller is to adjust online the two actuation values of a simulated car `[delta, a]` which stand for steering angle and throttle respectively.

## Reflection

### Model

The model used is a kinematic one which uses the following states:

* Vehicle position coordinates `x` and `y` 
* Orientation angle `psi`
* Vehicle velocity `v`
* Cross Track Error (the offset from the middle of the lane) `cte`
* Orientation angle error `epsi`

The actuation outputs of the model are:

* The steering angle `delta`
* The acceleration `a`

The model is described by the following equations:

```cpp
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
psi_[t+1] = psi[t] + (v[t] / Lf) * delta[t] * dt;
v_[t+1] = v[t] + a[t] * dt;
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;
epsi[t+1] = psi[t] - psides[t] + v[t] * (delta[t] / Lf) * dt;
```

Where `Lf` is the distance between the front wheels (actuation) and the car's center of the mass. The description of the parameter can be found [here](./src/MPC.cpp#L12)

The model tries to solve the problem of minimizing the "cost" of a function which is a combination of the following factors:

* Square sum of `cte`, `epsi` and `v`. (Also mentioned as reference state cost in the [code](./src/MPC.cpp#L78))
* Square sum of the actuations `delta` and `a`. (Also mentioned as actuation use cost in the [code](./src/MPC.cpp#L86))
* Square sum of the difference of two consecutive actuation values. (Also mentioned as sequential value gap cost in the [code](./src/MPC.cpp#L93))

A coefficient has been assigned to each of the factors described above in the [implementation](./src/MPC.cpp#L48) which affects its impact.

The values that had been tuned manually to optimize the performance of the simulated car inside the track can be found below:

```cpp
const double rfc_cte_coeff = 5000.0;      /*!< CTE reference state cost coefficient */
const double rfc_epsi_coeff = 5000.0;     /*!< Orientation error reference state cost coefficient */
const double rfc_v_coeff = 1.0;           /*!< Velocity reference state cost coefficient */
const double aus_delta_coeff = 3000.0;     /*!< Steering angle actuation use cost coefficient */
const double aus_a_coeff = 10.0;          /*!< Throttle actuation use cost coefficient */
const double svgc_delta_coeff = 25000000.0; /*!< Steering angle sequential value gap cost coeffiecient */
const double svgc_a_coeff = 5.0;       /*!< Throttle value sequential gap cost coefficient */
```

### Timestep Length and Elapsed Duration (N & dt)

The following values were chosen for the number of timesteps `N` and the timestep duration `dt` 

```cpp
size_t N = 15;    /*!< Number/Length of timesteps */
double dt = 0.05; /*!< Duration of each timestep */
```
With such configuration it was obvious also on the projection of the points on the simulator that the prediction horizon is well adapted both in straight lines and cornering with a high vehicle speed. The timestep resolution remained unchanged throughtout the complete implementation at 50ms and the only change was between the points. The first configuration was with 25 points but this created issues during the steep corners on the prediction so they had to be reduced to a value that it fits better with dynamic behavior of turning. 

### Polynomial Fitting and MPC Preprocessing

The simulator provides the waypoints on the map coordinate system. The `globalToVehicleCoordTransform` function implemented [here](./src/main.cpp#L69) translates the coordinates to the vehicle system. In order to fit the translated waypoints the `polyfit` function provided is used to create a 3rd degree polynomial as it was instructed in the classroom. The polynomial is used by the solver as the reference trajectory and for the calculation of `cte` and `epsi`

### MPC with Latency

The latency is taken into consideration during preprocessing. Instead of the initial values given by the simulator the actuation delay is injected in the calculation of the initial state [here](./src/main.cpp#L140).

## Simulation Result

The result of the controller at the simulation track can be found in this [video](./video/Final.mp4)


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

