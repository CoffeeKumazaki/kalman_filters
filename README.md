# Kalman Filters
This is a C++ implementation of the Kalman filter and its extension, the extended Kalman filter (EKF) and the unscented Kalman filter (UKF).

## Kalman Filter

## Extended Kalman Filter (EKF)

The extended Kalman filter is an extension of the Kalman filter for nonlinear systems.  
This technique linearizes a model at a working point using Taylor series expansion. 

```c++
#include <ekf.hpp>

// model  : system model (sub-class of SystemModel class)
// P      : initial covariance matrix
ExtendedKF ekf(model, P);

// x0: initial state
ekf.init(x0);

// u: control vector
ekf.predict(u);

// x: previous state vector
ekf.update(x);

// get estimated state and covariance matrix
Eigen::VectorXd x_est = ekf.get_state();
Eigen::MatrixXd P_est = ekf.get_cov();
```

![ekf_gif](doc/ekf.gif)

## Unscented Kalman Filter (UKF)

The extended Kalman filter requires differentiable models and gives poor performance in highly nonlinear systesm.  
The unscented Kalman filter uses a deterministic sampling technique known as the unscented transformation to calculate statistics around the mean. This technique does not require differentiability of models.

```c++
// it's almost the same with the ekf.
#include <ukf.hpp>

// model  : system model (sub-class of SystemModel class)
// P      : initial covariance matrix
// scale  : scaling parameter for adjust the effects of third-order momentum. [optional]
UnscentedKF ukf(model, P, scale);

// x0: initial state
ukf.init(x0);

// u: control vector
ukf.predict(u);

// x: previous state vector
ukf.update(x);

// get estimated state and covariance matrix
Eigen::VectorXd x_est = ukf.get_state();
Eigen::MatrixXd P_est = ukf.get_cov();
```

![ukf_gif](doc/ukf.gif)