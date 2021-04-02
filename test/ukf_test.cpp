#include <stdafx.hpp>
#include <kalman_filters.hpp>
#include <ukf.hpp>
#include <system_simulator.hpp>
#include <plot_helper.hpp>

int main(int argc, char const *argv[]) {

	// state: x, y, yaw, v
	Eigen::VectorXd x0(4);
	// x0.setZero();
	x0 << 0.0, 0.0, deg2rad(1.0), 1.0;

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
	std::shared_ptr<SystemModelA> model = std::make_shared<SystemModelA>(x0, R, Q, dt);
	SystemSimulator sim(model);
	sim.init();
	Eigen::MatrixXd P(4, 4);
	P << 1.0, 0.0, 0.0, 0.0,
			 0.0, 1.0, 0.0, 0.0, 
			 0.0, 0.0, 1.0, 0.0, 
			 0.0, 0.0, 0.0, 1.0;
	UnscentedKF ukf(model, P, 1.0);
	ukf.init(x0);
	double t = 0;

	// u: v, yaw
	Eigen::VectorXd u(2);
	u << 0.1, deg2rad(4.0);

	std::vector<Eigen::VectorXd> res;
	std::vector<Ellipse> es;
	while (t < 60.0) {
		sim.step(u);
		ukf.predict(u);
		ukf.update(sim.obs_results.back());
		res.push_back(ukf.get_state());
		Ellipse e;
		get_covariance_ellipse(ukf.get_cov().block(0, 0, 2, 2), e);
		es.push_back(e);
		t += dt;
	}

	for (size_t i = 0; i < sim.true_results.size(); i++) {
		std::cout << dt*i << ", " 
							<< sim.true_results[i](0) << ", " << sim.true_results[i](1) << ", "
							<< sim.obs_results[i](0) << ", " << sim.obs_results[i](1) << ", "
							<< res[i](0) << ", " << res[i](1) << ", " 
							<< es[i].major_ax << ", " << es[i].minor_ax << ", " << es[i].angle
							<< std::endl;
	}

	return 0;
}
