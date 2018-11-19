#include <iostream>
#include <cassert>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  assert(!estimations.empty());
  assert(estimations.size() == ground_truth.size());

  for (auto i = 0; i < estimations.size(); ++i)
  {
    VectorXd e = estimations[i];
    VectorXd g = ground_truth[i];

    VectorXd diff = e - g;
    VectorXd diff_2 = diff.array() * diff.array();
    rmse += diff_2;
  }

  rmse /= estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}