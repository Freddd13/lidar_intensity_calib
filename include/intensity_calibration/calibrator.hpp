/***
 * @Date: 2022-09-27 20:25:58
 * @LastEditors: Freddd13
 * @LastEditTime: 2022-09-29 19:13:19
 * @Description:
 * @yzdyzd13@gmail.com
 */
#pragma once
#include "intensity_calibration/kumo_algorithms.h"
#include "intensity_calibration/model/beam_model.hpp"
#include "intensity_calibration/model/cell_model.h"
#include "intensity_calibration/model/mesurement.h"
#include "intensity_calibration/params.hpp"

template <class PointType>
class Calibrator {
 public:
  Calibrator() = delete;
  Calibrator(Params* params);
  ~Calibrator();
  void LoadCloud();
  void Init();
  Measurements LoadMeasurement(float voxel_size);
  void InitCellModel();
  void InitBeamModel();

  BeamMappings Run();
  double EStep();
  double MStep();
  void SaveCalibResult(BeamMappings mappings);

 private:
  typename pcl::PointCloud<PointType>::Ptr cloud_;
  // boost::shared_ptr<Params> params_;
  Params* params_;
  Measurements measurments_;
  BeamModels beam_models_;
  CellModels cell_models_;
  int num_beams_ = 0;
  int num_voxels_ = 0;
  std::vector<double> loss_e_;
  std::vector<double> loss_m_;
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
  // TODO check fields
  // TODO why downsample not working???
  pcl::copyPointCloud(*cloud, *cloud_);
  for (auto& pt : cloud->points) {
    // if (pt.normal_y <= params_->max_distance_) {  // for XYZIN, normal_x -> scan_id, normal_y ->distance
    //   cloud_->push_back(pt);
    // }
    if (pt.intensity <= params_->max_intensity_) {
      cloud_->push_back(pt);
    }
    num_beams_ = std::max(num_beams_, (int)pt.normal_x);
  }
  LOG(INFO) << "Num of points read from pcd: " << cloud->size() << ", keep " << cloud_->size();
  LOG(INFO) << "Num of beams: " << ++num_beams_;
}

// Init. Read Measurement and init models
template <class PointType>
void Calibrator<PointType>::Init() {
  this->LoadCloud();
  measurments_ = this->LoadMeasurement(params_->voxel_size_);
  this->InitCellModel();
  this->InitBeamModel();
  // LOG(INFO) << "Num of points read from pcd: " << cloud->size() << ", keep " << cloud_->size();
}

// load measurments
template <class PointType>
Measurements Calibrator<PointType>::LoadMeasurement(float voxel_size) {
  // Put in bins
  pcl::VoxelGrid<PointType> grid;
  grid.setInputCloud(cloud_);
  grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  grid.setSaveLeafLayout(true);
  grid.setDownsampleAllData(false);
  typename pcl::PointCloud<PointType>::Ptr tmpcloud(new pcl::PointCloud<PointType>);
  grid.filter(*tmpcloud);

  // Now put in measurement
  Measurements results;
  // std::vector<size_t> counter(257,0);
  for (const auto pt : cloud_->points) {
    int a = static_cast<int>(pt.intensity);
    int b = static_cast<int>(pt.normal_x);
    int k = grid.getCentroidIndexAt(grid.getGridCoordinates(pt.x, pt.y, pt.z));
    if (a >= params_->max_intensity_) {
      continue;
    }
    CHECK(k >= 0) << "Did not find corresponding voxel";
    CHECK(a <= params_->max_intensity_ && a >= 0) << "Measured intensity is wrong value";
    CHECK(b >= 0) << "Beam id is wrong";
    results.emplace_back(a, b, k);
  }
  LOG(INFO) << "Gathered measurements " << results.size();
  cloud_.swap(tmpcloud);

  return results;
}

// cell model
template <class PointType>
void Calibrator<PointType>::InitCellModel() {
  cell_models_.resize(cloud_->size());
  for (auto& cell_model : cell_models_) {
    cell_model = Eigen::Matrix<double, Eigen::Dynamic, 1>();
    cell_model.resize(params_->max_intensity_, 1);
    cell_model.setOnes();
    cell_model = cell_model / params_->max_intensity_;
    // LOG(INFO) << cell_model.transpose();
  }

  LOG(INFO) << "Init CellModel";
}

// beam model
template <class PointType>
void Calibrator<PointType>::InitBeamModel() {
  // vlp16 for example:
  // 16 beams, each beam has 255 original value, each value has 255 possible real value
  // Here, assuming given original value == map value == "m", m * a -> 255 * 255
  for (int i = 0; i < num_beams_; i++) {
    beam_models_.emplace_back(params_->max_intensity_, params_->std_var_, params_->eps_);
  }
  LOG(INFO) << "Init BeamModel";
}

///// EM /////
// E step
template <class PointType>
double Calibrator<PointType>::EStep() {
  LOG(INFO) << "E Step, update voxel models";
  auto buffered = cell_models_;
  for (auto& cell_model : cell_models_) cell_model.setZero();
  for (const auto& m : measurments_) {
    cell_models_.at(m.k) += beam_models_.at(m.b).atLog(m.a);
  }

  if (params_->use_original_em_) {
    // original from Guo.
    for (auto& cell_model : cell_models_) {
      auto val = cell_model.maxCoeff();
      for (int i = 0; i < params_->max_intensity_; i++) {
        if (cell_model(i) - val >= std::log(params_->original_em_precision_) - std::log(params_->max_intensity_)) {
          cell_model(i) = std::exp(cell_model(i) - val);
        } else {
          cell_model(i) = 0;
        }
      }
    }
  } else {
    // simple
    for (auto& cell_model : cell_models_) {
      // Eigen::MatrixXd::Index max_intensity, mmm;
      // auto val = cell_model.maxCoeff(&max_intensity, &mmm);  // 对于这个cell,最有可能的Intensity的概率值
      // LOG(INFO) << "most prob i: " << mmm << " prob: " << val;
      for (int i = 0; i < params_->max_intensity_; i++) {
        cell_model(i) = std::exp(cell_model(i));
      }
    }
  }
  double diff = 0.0;
  for (uint i = 0; i < cell_models_.size(); i++) {
    // LOG(INFO) << "sum " << cell_model.at(i).sum();
    // LOG(INFO) << "sum " << cell_model.at(i).transpose();
    if (cell_models_.at(i).sum() == 0) {
      LOG(INFO) << "empty, pass";
    } else {
      cell_models_.at(i) /= cell_models_.at(i).sum();  // 归一化
    }
    double err = (cell_models_.at(i) - buffered.at(i)).norm();
    diff += err;
  }

  double res = diff / cell_models_.size();
  loss_e_.push_back(res);
   LOG(WARNING) << "Current E STEP averaged error" << res;
  return res;
}

// M step
template <class PointType>
double Calibrator<PointType>::MStep() {
  LOG(INFO) << "M Step, update beam models";
  auto buffered = beam_models_;
  BeamCountings countings(beam_models_.size());
  for (auto& counting : countings) {
    counting = BeamCounting(params_->max_intensity_, params_->max_intensity_);
    counting.setZero();
  }
  for (const auto& m : measurments_) {
    countings.at(m.b).col(m.a) += cell_models_.at(m.k);
  }
  double diff = 0;
  for (uint i = 0; i < countings.size(); i++) {
    CHECK_EQ(countings.at(i).rows(), params_->max_intensity_);
    beam_models_.at(i) = BeamModel(countings.at(i));
    diff += (beam_models_.at(i).probability_ - buffered.at(i).probability_).norm();
  }

  double res = diff / countings.size();
  loss_m_.push_back(res);
  LOG(WARNING) << "Current M STEP averaged error" << res;
  return res;
}

// run calib!!!
template <class PointType>
void Calibrator<PointType>::SaveCalibResult(BeamMappings mappings) {
  std::string::size_type iPos = (params_->pcd_path_).find_last_of('/') + 1;
  std::string filename_pcd = (params_->pcd_path_).substr(iPos, (params_->pcd_path_).length() - iPos);
  filename_pcd = filename_pcd.substr(0, filename_pcd.rfind("."));

  // round(x * 100) / 100.0.
  auto name_prefix = params_->result_path_;
  auto name_pcd_in_res = filename_pcd;
  auto name_voxel = "_voxel" + std::to_string(round(params_->voxel_size_ * 100) / 100.0);
  auto name_dis = "_dis" + std::to_string(round(params_->max_distance_ * 100) / 100.0);
  auto name_intensity = "_intensity" + std::to_string(round(params_->max_intensity_ * 100) / 100.0);
  auto name_var = "_var" + std::to_string(round(params_->std_var_ * 100) / 100.0);
  auto name_eps = "_eps" + std::to_string(round(params_->eps_ * 100) / 100.0);
  auto name_loss = "_loss" + std::to_string(params_->value_converge_);
  auto name_suffix = ".txt";

  auto filename_res = name_prefix + name_pcd_in_res + name_voxel + name_dis + name_intensity + name_var + name_eps +
                      name_loss + name_suffix;
  std::ofstream file(filename_res);
  if (file.is_open()) {
    // file << mappings.size() << " " << mappings.at(0).cols() << "\n";
    for (const auto mapping : mappings) {
      file << mapping << '\n';
    }
  }

  auto filename_loss = name_prefix + "loss.txt";
  std::ofstream file_loss(filename_loss);
  if (file_loss.is_open()) {
    // file << mappings.size() << " " << mappings.at(0).cols() << "\n";
    for (const auto loss : loss_m_) {
      file_loss << " " << loss;
    }
    file_loss << std::endl;
    for (int i = 0; i< (int)loss_m_.size(); i++) {
      file_loss << " " << loss_e_.at(i);
    }
  }
}

// run calib!!!
template <class PointType>
BeamMappings Calibrator<PointType>::Run() {
  LOG(INFO) << "Start Calib!!!";
  int count = 0;
  while (EStep() > params_->value_converge_ && count < params_->max_em_epoch_) {
    MStep();
    LOG(INFO) << "============"
              << "epoch " << count << "============";
    count++;
  }
  BeamMappings mappings;
  for (const auto& beam_model : beam_models_) {
    mappings.emplace_back(beam_model.GetMapping());
  }
  return mappings;
}