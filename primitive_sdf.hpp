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

class CylinderSDF : public SDFBase {
 public:
  double radius_;
  double height_;
  CylinderSDF(double radius, double height, const Pose& tf)
      : SDFBase(tf), radius_(radius), height_(height) {}

 private:
  Values evaluate_in_local_frame(const Eigen::Matrix3Xd& p) const override {
    Eigen::VectorXd&& d = (p.topRows(2).colwise().norm().array() - radius_);
    auto outside_distance = (d.cwiseMax(0.0)).colwise().norm();
    auto inside_distance = d.cwiseMin(0.0).cwiseAbs().colwise().maxCoeff();
    Eigen::VectorXd&& height_distance =
        (p.row(2).cwiseAbs().array() - (height_ / 2.0)).cwiseMax(0.0);
    return (outside_distance + inside_distance).cwiseMax(height_distance);
  }
};

class SphereSDF : public SDFBase {
 public:
  double radius_;

  SphereSDF(double radius, const Pose& tf) : SDFBase(tf), radius_(radius) {}

 private:
  Values evaluate_in_local_frame(const Eigen::Matrix3Xd& p) const override {
    return (p.colwise().norm().array() - radius_);
  }
};

}  // namespace primitive_sdf
