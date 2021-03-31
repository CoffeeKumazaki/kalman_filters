#include <stdafx.hpp>
#include <kalman_filters.hpp>
#include <ekf.hpp>
#include "system_simulator.hpp"

#define deg2rad(a) ((a)/180.0 * M_PI)

int main(int argc, char const *argv[]) {

	SystemSimulator sim;
	// state: x, y, yaw, v
	Eigen::VectorXd x0(4);
	// x0.setZero();
	x0 << 0.0, 0.0, deg2rad(1.0), 10.0;

	Eigen::MatrixXd Q(4, 4);
	Q << 0.0, 0.0, 0.0, 0.0,
			 0.0, 0.0, 0.0, 0.0,
			 0.0, 0.0, deg2rad(0.1), 0.0,
			 0.0, 0.0, 0.0, 0.01;

	Q = Q * Q;

	Eigen::MatrixXd R(2, 2);
	R << 0.5, 0.0, 
			 0.0, 0.5;

	R = R * R;
	double dt = 1.0 / 30.0;
	sim.init_model(x0, R, Q, dt);
	double t = 0;

	// u: v, yaw
	Eigen::VectorXd u(2);
	u << 0.0, deg2rad(2.0);
	while (t < 60.0) {
		sim.step(u);
		t += dt;
	}

	for (size_t i = 0; i < sim.true_results.size(); i++) {
		std::cout << dt*i << ", " 
							<< sim.true_results[i](0) << ", " << sim.true_results[i](1) << ", "
							<< sim.obs_results[i](0) << ", " << sim.obs_results[i](1)
							<< std::endl;
	}

	return 0;
}
