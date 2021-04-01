#pragma once

#include <Eigen/Dense>

struct Ellipse {
  double center_x, center_y;
  double angle;
  double major_ax, minor_ax;
};

void get_covariance_ellipse(const Eigen::MatrixXd& cov, Ellipse& ellipse) {

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov);
  if (eigensolver.info() != Eigen::Success) abort();

  auto max_eigenval = eigensolver.eigenvalues()(1);
  auto min_eigenval = eigensolver.eigenvalues()(0);
  auto max_eigenvec = eigensolver.eigenvectors().col(1);

  ellipse.major_ax = sqrt(max_eigenval)*2.0;
  ellipse.minor_ax = sqrt(min_eigenval)*2.0;
  ellipse.angle = rad2deg(atan(max_eigenvec(1)/max_eigenvec(0)));
  // std::cout << ellipse.major_ax << ", " << ellipse.minor_ax << ", " << (ellipse.angle) << "" << std::endl;
}
