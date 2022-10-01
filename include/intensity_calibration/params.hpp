/***
 * @Date: 2022-09-27 19:02:30
 * @LastEditors: Freddd13
 * @LastEditTime: 2022-09-27 19:56:07
 * @Description:
 * @yzdyzd13@gmail.com
 */
#pragma once
#include "intensity_calibration/kumo_algorithms.h"

struct Params {
 public:
  Params(YAML::Node config) {
    log_path_ = config["log_path"].as<std::string>();
    result_path_ = config["result_path"].as<std::string>();
    pcd_path_ = config["pcd_path"].as<std::string>();
    voxel_size_ = config["voxel_size"].as<float>();
    std_var_ = config["std_var"].as<float>();
    eps_ = config["eps"].as<float>();
    max_intensity_ = config["max_intensity"].as<int>();
    value_converge_ = config["value_converge"].as<float>();
    use_original_em_ = config["use_original_em"].as<bool>();
    original_em_precision_ = config["original_em_precision"].as<double>();
    max_em_epoch_ = config["max_em_epoch"].as<int>();
  }
  // ~Params() {}

  std::string log_path_;
  std::string result_path_;
  std::string pcd_path_;
  float voxel_size_;
  float max_distance_;
  float std_var_;
  float eps_;
  float value_converge_;
  int max_intensity_;
  bool use_original_em_;
  double original_em_precision_;
  int max_em_epoch_;
};
