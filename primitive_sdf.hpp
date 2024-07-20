#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

namespace primitive_sdf {

using Points = Eigen::Matrix3Xd;
using Values = Eigen::VectorXd;

class Pose {
 public:
  Pose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
      : position_(position), rot_inv_(rotation.inverse()) {}

  Points transform_points(const Points& p) const {
    return rot_inv_ * (p.colwise() - position_);
  }

 private:
  Eigen::Vector3d position_;
  Eigen::Matrix3d rot_inv_;
};

class SDFBase {
 public:
  SDFBase(const Pose& tf) : tf_(tf) {}

  Values evaluate(const Points& p) const {
    auto p_local = tf_.transform_points(p);
    return evaluate_in_local_frame(p_local);
  }

 private:
  Pose tf_;

 protected:
  virtual Values evaluate_in_local_frame(const Points& p) const = 0;
};

class BoxSDF : public SDFBase {
 public:
  Eigen::Vector3d width_;

  BoxSDF(const Eigen::Vector3d& width, const Pose& tf)
      : SDFBase(tf), width_(width) {}

 private:
  Values evaluate_in_local_frame(const Eigen::Matrix3Xd& p) const override {
    auto half_width = width_ / 2.0;
    auto d = p.cwiseAbs().colwise() - half_width;
    auto outside_distance = (d.cwiseMax(0.0)).colwise().norm();
    auto inside_distance = d.cwiseMin(0.0).colwise().maxCoeff();
    Values vals = outside_distance + inside_distance;
    return vals;
  }
};

// class CylinderSDF : public SDFBase<TranslationTransform>
// {
// public:
//   double radius_;
//   double height_;
//   CylinderSDF(double radius, double height, const FullTransform& tf) :
//   CylinderSDF(radius, height, TranslationTransform(tf)) {} CylinderSDF(double
//   radius, double height, const TranslationTransform& tf) :
//   SDFBase<TranslationTransform>(tf), radius_(radius), height_(height) {}
//
// private:
//   Values evaluate_in_local_frame(const Eigen::Matrix3Xd& p) const override {
//     auto d = (p.topRows(2).colwise().norm() - radius_).eval();
//     auto outside_distance = (d.cwiseMax(0.0)).colwise().norm();
//     auto inside_distance = d.cwiseMin(0.0).cwiseAbs().colwise().maxCoeff();
//     auto height_distance = (p.row(2).cwiseAbs() - height_
//     / 2.0).cwiseMax(0.0); Values vals = (outside_distance +
//     inside_distance).cwiseMax(height_distance); return vals;
//   }
// };

}  // namespace primitive_sdf
