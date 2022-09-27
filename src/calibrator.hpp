/***
 * @Date: 2022-09-27 20:25:58
 * @LastEditors: Freddd13
 * @LastEditTime: 2022-09-27 20:41:15
 * @Description:
 * @yzdyzd13@gmail.com
 */
#pragma once
#include "intensity_calibration/kumo_algorithms.h"
#include "params.hpp"

template <class PointType>
class Calibrator {
 public:
  Calibrator() = delete;
  Calibrator(Params* params);
  ~Calibrator();
  void LoadCloud();

 private:
  typename pcl::PointCloud<PointType>::Ptr cloud_;
  // boost::shared_ptr<Params> params_;
  Params* params_;
};

// constructor
template <class PointType>
Calibrator<PointType>::Calibrator(Params* params) {
  // params_.reset(params); // 这里使用智能指针将导致内存释放两次
  params_ = params;
  cloud_.reset(new pcl::PointCloud<PointType>());
  LOG(INFO) << "test params" << params->log_path_;
}

// deconstructor
template <class PointType>
Calibrator<PointType>::~Calibrator() {}

// load cloud
template <class PointType>
void Calibrator<PointType>::LoadCloud() {
  typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::io::loadPCDFile(params_->pcd_path_, *cloud);
  //TODO check fields
  //TODO why downsample not working???
  pcl::copyPointCloud(*cloud, *cloud_);
  // for(auto &pt : cloud->points) {
  //   if (pt.normal_y <= params_->max_distance_) {  // for XYZIN, normal_x -> scan_id, normal_y ->distance
  //     cloud_->push_back(pt);
  //   }
  // }
  LOG(INFO) << "Num of points read from pcd: " << cloud->size() << ", keep " << cloud_->size();
}
