/*** 
 * @Date: 2022-09-27 21:44:18
 * @LastEditors: Freddd13
 * @LastEditTime: 2022-09-30 11:05:25
 * @Description: 
 * @yzdyzd13@gmail.com
 */

#include "intensity_calibration/kumo_algorithms.h"
#include "intensity_calibration/params.hpp"

int main(int argc, char** argv) {
  // init gflags
  google::ParseCommandLineFlags(&argc, &argv, true);

  // init glog
  // for non-ros
  YAML::Node config = YAML::LoadFile("../config/config.yaml");
  Params params(config);
  std::string log_dir = params.log_path_;

  // for ros
  // std::string log_dir = ros::package::getPath("intensity_calibration") + "/log/";
  // ROS_WARN("Log dir: %s", log_dir.c_str());
  // FLAGS_log_dir = log_dir;

  FLAGS_log_dir = log_dir;
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "log dir: " << log_dir;

  // read cloud
  pcl::PointCloud<KPointXYZIN>::Ptr cloud(new pcl::PointCloud<KPointXYZIN>);
  pcl::PointCloud<KPointXYZIN>::Ptr after_cloud(new pcl::PointCloud<KPointXYZIN>);
  pcl::io::loadPCDFile(params.pcd_path_, *cloud);

  pcl::UniformSampling<KPointXYZIN> form;  // 创建均匀采样对象
  form.setInputCloud(cloud);               //设置输入点云
  form.setRadiusSearch(0.1f);              //设置半径大小,单位:m
  form.filter(*after_cloud);
  pcl::io::savePCDFileASCII(
      "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/sample/test_uniform_0.1.pcd",
      *after_cloud);

  after_cloud->clear(); //TODO voxel可能不支持xyzin????(Normal被舍了)
  pcl::VoxelGrid<KPointXYZIN> sor;       //创建体素网格采样处理对象
  sor.setInputCloud(cloud);              //设置输入点云
  sor.setLeafSize(0.1f, 0.1f, 0.1f);  //设置体素大小，单位：m
  sor.filter(*after_cloud);           //进行下采样
  pcl::io::savePCDFileASCII(
      "/home/yxt/code/slam/intensity_calib_fred_ws/src/intensity_calibration/sample/test_voxel_0.1.pcd", *after_cloud);

  return 0;
}