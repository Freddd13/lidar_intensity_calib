#pragma once
#include "intensity_calibration/kumo_algorithms.h"
#include "cell_model.h"
#include "beam_model.h"

using BeamMapping = Eigen::RowVectorXi;  //< Map to 0 till 255

struct PDF {
  PDF() : mu_(0.0), var_(1.0) {}
  PDF(const double& mu, const double& var) : mu_(mu), var_(var) {}
  double at(const double& x) { return std::exp(-0.5 * (x - mu_) * (x - mu_) / var_ / var_); }
  double mu_, var_;
};

struct BeamModel {
  BeamModel(Eigen::MatrixXd in) {
    probability_ = in;
    this->Normalize();
    log_probability_ = probability_.array().log().matrix();
  }

  BeamModel(int size, double var = 1.0, double epsilon = 0.1) {
    probability_ = Eigen::MatrixXd(size, size);
    for (int i = 0; i < size; i++) {  // m
      PDF pdf(static_cast<double>(i), var);
      for (int j = 0; j < size; j++) {  // a
        probability_(i, j) = pdf.at(static_cast<double>(j)) + epsilon;
      }
    }
    this->Normalize();  // TODO
    log_probability_ = probability_.array().log().matrix();
    // LOG(INFO) << probability_;
  }

  void Normalize() {
    // TODO check?
    Eigen::VectorXd row_wise_sum = probability_.rowwise().sum();
    probability_.array().colwise() /= row_wise_sum.array();
    CHECK_NEAR(probability_.row(0).sum(), 1.0, 1e-4);
  }

  CellModel atLog(int measured) { return log_probability_.col(measured); }

  Eigen::MatrixXd probability_, log_probability_;

  BeamMapping GetMapping() const {
    BeamMapping result(probability_.rows());
    CHECK_NEAR(probability_.row(0).sum(), 1.0, 1e-4);
    for (uint i = 0; i < probability_.rows(); i++) {
      auto val = probability_.col(i).maxCoeff(&result(i));
      if (val == 0) result(i) = i;
      // LOG(INFO) << " Most likely ground truth at measured " << i <<
      //         " is " << result(i) << " at probability " << val;
    }
    return result;
  }
};
using BeamMappings = std::vector<BeamMapping>;

using BeamModels = std::vector<BeamModel>;
using BeamCounting = Eigen::MatrixXd;
using BeamCountings = std::vector<BeamCounting>;
