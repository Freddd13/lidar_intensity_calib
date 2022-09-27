/***
 * @Date: 2022-09-27 20:24:26
 * @LastEditors: Freddd13
 * @LastEditTime: 2022-09-27 20:24:27
 * @Description:
 * @yzdyzd13@gmail.com
 */

#include "calibrator.hpp"

template <class PointType>
Calibrator<PointType>::Calibrator(Params* params) {
  params_.reset(params);
  cloud_.reset(new pcl::PointCloud<PointType>());
  LOG(INFO) << "test params" << params->log_path_;
}

template <class PointType>
Calibrator<PointType>::~Calibrator() {}
