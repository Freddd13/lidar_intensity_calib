#pragma once
#ifndef KUMO_ALGORITHMS_H
#define KUMO_ALGORITHMS_H
// #define PCL_NO_PRECOMPILE
#include "intensity_calibration/common.h"

// new point type

struct PointXYZIR {
  // Standard Point
  PCL_ADD_POINT4D;

  union {
    float data_c[4];  // ensuring SSE alignment
    struct {
      unsigned char ring;        // This is 8 bit, number of ring
      unsigned short intensity;  // This is 16 bit
      float range;
    };
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(unsigned char, ring, ring_number)(
                                                  unsigned short, intensity, intensity)(float, range, range))

struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

struct PointXYZIARPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float ratio;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIARPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, ratio,
                                                                                                       ratio)(
                                      float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

struct PointXYZIA {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float ratio;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIA, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(float, ratio, ratio))

typedef PointXYZIRPYT KPointXYZIRPYT;
typedef PointXYZIARPYT KPointXYZIARPYT;
typedef PointXYZIA KPointXYZIA;
typedef PointXYZIR KPointXYZIR;
typedef pcl::PointXYZINormal KPointXYZIN;
typedef pcl::PointXYZ KPointXYZ;
typedef pcl::PointXYZI KPointXYZI;
typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZI> CloudXYZI;
typedef pcl::PointCloud<PointXYZIRPYT> CloudXYZIRPYT;
typedef pcl::PointCloud<PointXYZIARPYT> CloudXYZIARPYT;
typedef pcl::PointCloud<PointXYZIA> CloudXYZIA;
typedef pcl::PointCloud<PointXYZIR> CloudXYZIR;
typedef pcl::PointCloud<pcl::PointXYZINormal> CloudXYZIN;
// rad&deg transfrom

// Rotation matrix & roll pitch yaw transform.
class RETransform {
 public:
  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr) {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }
};

#endif