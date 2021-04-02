# Kalman Filters
This is a C++ implementation of the Kalman filter and its extension, the extended Kalman filter (EKF) and the unscented Kalman filter (UKF).

## Kalman Filter

## Extended Kalman Filter (EKF)

The extended Kalman filter is an extension of the Kalman filter for nonlinear systems.  
This technique linearizes a model at a working point using Taylor series expansion. 

![ekf_gif](doc/ekf.gif)

## Unscented Kalman Filter (UKF)

The extended Kalman filter requires differentiable models and gives poor performance in highly nonlinear systesm.  
The unscented Kalman filter uses a deterministic sampling technique known as the unscented transformation to calculate statistics around the mean. This technique does not require differentiability of models.

![ukf_gif](doc/ukf.gif)