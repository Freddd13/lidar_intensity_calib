/***
 * @Author: Freddd13
 * @Date: 2022-04-22 10:35:52
 * @LastEditors: Freddd13
 * @LastEditTime: 2022-08-09 13:36:14
 * @FilePath: /kumo_slam/include/kumo_slam/common.h
 * @Description:
 * @yzdyzd13@gmail.com
 * @Copyright (c) 2022 by Freddd13, All Rights Reserved.
 */
#pragma once

#include <ceres/ceres.h>
#include <opencv/cv.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>

#include "yaml-cpp/yaml.h"
// #include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/search/impl/search.hpp>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// using namespace std;
// using namespace Eigen;
// using namespace pcl;
