#include <stdafx.hpp>
#include <ukf.hpp>

UnscentedKalmanFilter::UnscentedKalmanFilter(
  std::shared_ptr<SystemModel> _model, 
  const Eigen::MatrixXd& _P, 
  double scale /*= 0*/
)
: model(_model)
, P(_P)
, P0(_P)
, x_hat(P.rows())
, lambda(scale)
{
  wc.reserve(2 * P.rows() + 1);
  ws.reserve(2 * P.rows() + 1);

  ws.push_back(lambda / (P.rows() + lambda));
  wc.push_back(lambda / (P.rows() + lambda));

  for (size_t i = 1; i < 2*P.rows()+1; i++) {
    double val = 1.0 / (2.0 * (lambda + P.rows()));
    ws.push_back(val);
    wc.push_back(val);
  }
}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {

}

void UnscentedKalmanFilter::init() {

  P = P0;
  x_hat.setZero();
  I = Eigen::MatrixXd::Identity(P.rows(), P.rows());
}

void UnscentedKalmanFilter::init(const Eigen::VectorXd& x) {

  init();
  x_hat = x;
}

void UnscentedKalmanFilter::ut(const Eigen::VectorXd& _x, const Eigen::MatrixXd& _P, std::vector<Eigen::VectorXd>& sigmaPt) {

  Eigen::LLT<Eigen::MatrixXd> chol(_P);
  Eigen::MatrixXd L = chol.matrixL();

  sigmaPt.clear();
  sigmaPt.reserve(2 * _P.rows() + 1);

  sigmaPt.push_back(x_hat);
  for (size_t i = 0; i < _P.rows(); i++) {
    Eigen::VectorXd preSigmaPt1 = _x + (sqrt(P.rows() + lambda) * L).col(i);
    Eigen::VectorXd preSigmaPt2 = _x - (sqrt(P.rows() + lambda) * L).col(i);
    sigmaPt.push_back(preSigmaPt1);
    sigmaPt.push_back(preSigmaPt2);
  }
}

void UnscentedKalmanFilter::predict(const Eigen::VectorXd& u) {

  std::vector<Eigen::VectorXd> sigmaPt;
  ut(x_hat, P, sigmaPt);

  Eigen::VectorXd x_new(x_hat.rows());
  x_new.setZero();
  for (size_t i = 0; i < 2*P.rows() + 1; i++) {
    sigmaPt[i] = model->DynamicsModel(sigmaPt[i], u);
    x_new += ws[i] * sigmaPt[i];
  }
  Eigen::MatrixXd p_new(P.rows(), P.rows());
  p_new.setZero();
  for (size_t i = 0; i < 2*P.rows() + 1; i++) {
    p_new += wc[i] * (sigmaPt[i] - x_new) * (sigmaPt[i] - x_new).transpose();
  }

  x_hat = x_new;
  P = p_new + model->Q;

}

void UnscentedKalmanFilter::update(const Eigen::VectorXd& y) {

  std::vector<Eigen::VectorXd> sigmaPt;
  ut(x_hat, P, sigmaPt);

  std::vector<Eigen::VectorXd> gamma;
  gamma.resize(sigmaPt.size());

  Eigen::VectorXd z(y.rows());
  z.setZero();
  for (size_t i = 0; i < 2*P.rows() + 1; i++) {
    gamma[i] = model->ObservationModel(sigmaPt[i]); 
    z += ws[i] * gamma[i];
  }

  Eigen::MatrixXd P_obs(z.rows(), z.rows()), P_est(P.rows(), z.rows());
  P_obs.setZero();
  P_est.setZero();
  for (size_t i = 0; i < 2*P.rows() + 1; i++) {
    P_obs += wc[i] * (gamma[i] - z) * (gamma[i] - z).transpose();
    P_est += wc[i] * (sigmaPt[i] - x_hat) * (gamma[i] - z).transpose();
  }

  K = P_est * (P_obs + model->R).inverse();
  x_hat += K * (y - z);
  P = P - K * P_est.transpose();
}

Eigen::VectorXd UnscentedKalmanFilter::get_state() { 
  return x_hat; 
}

Eigen::MatrixXd UnscentedKalmanFilter::get_cov() {
  return P;
}