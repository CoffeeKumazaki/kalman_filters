#pragma once
#include <system_model.hpp>

class SystemModelA : public SystemModel {

public:
  SystemModelA (
    const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& _R, // Measurement noise covariance.
    const Eigen::MatrixXd& _Q, // Process noise covariance.
    double _dt = 1.0/30.0
  )
  : SystemModel(x0, _R, _Q, _dt)
  {
  }

  virtual Eigen::VectorXd DynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
    /*
		// std::cout << "DynamicsModel" << std::endl;
    static Eigen::MatrixXd f(4, 4);
    f << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0;

    Eigen::MatrixXd b(4, 2);
    b << dt*cos(x(2)), 0.0,
         dt*sin(x(2)), 0.0,  
         0.0, dt,
         1.0, 0.0;

    return f * x + b * u;
    */
    double px = x(0);
    double py = x(1);
    double yaw = x(2);
    double v = x(3);

    double yaw_v = u(1);
    double acc = u(0);

    Eigen::VectorXd newX(4);
    newX(0) = px + v*dt*cos(yaw);
    newX(1) = py + v*dt*sin(yaw);
    newX(2) = yaw + yaw_v*dt;
    newX(3) = v + acc * dt;

    return newX;
  }

  virtual Eigen::VectorXd ObservationModel(const Eigen::VectorXd &x) {

		// std::cout << "ObservationModel" << std::endl;
    static Eigen::MatrixXd h(2, 4);
    h << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0;

    return h * x;
  }

  virtual Eigen::MatrixXd JacobDynamicsModel(const Eigen::VectorXd &x, const Eigen::VectorXd &u) {
    double yaw = x(2);
    double v = u(0);

    Eigen::MatrixXd jF(4, 4);
    jF << 1.0, 0.0, -dt*v*sin(yaw), dt*v*cos(yaw),
          0.0, 1.0,  dt*v*cos(yaw), dt*v*sin(yaw),
          0.0, 0.0, 1.0, 0.0, 
          0.0, 0.0, 0.0, 1.0;

    return jF;
  }

  virtual Eigen::MatrixXd JacobObservationModel() {

    Eigen::MatrixXd jH(2, 4);
    jH << 1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0;

    return jH;
  }
};

class SystemSimulator {

public:
  SystemSimulator();
  ~SystemSimulator();

  void init_model(
    const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& _R, // Measurement noise covariance.
    const Eigen::MatrixXd& _Q, // Process noise covariance.
    double dt = 1.0/30.0
  );
  void step(Eigen::VectorXd& u);

private:
  std::shared_ptr<SystemModelA> model;

public:
  std::vector<Eigen::VectorXd> true_results;
  std::vector<Eigen::VectorXd> obs_results;
};