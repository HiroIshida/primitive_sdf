#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace primitive_sdf {

using Point = Eigen::Vector3d;
using Points = Eigen::Matrix3Xd;
using Values = Eigen::VectorXd;

class Pose {
 public:
  Pose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation)
      : position_(position), rot_inv_(rotation.inverse()) {}

  Points transform_points(const Points& p) const {
    return rot_inv_ * (p.colwise() - position_);
  }

  Point transform_point(const Point& p) const {
    return rot_inv_ * (p - position_);
  }

 private:
  Eigen::Vector3d position_;
  Eigen::Matrix3d rot_inv_;
};

class SDFBase {
 public:
  using Ptr = std::shared_ptr<SDFBase>;
  // for ease of binding to python, we name different functions
  virtual Values evaluate_batch(const Points& p) const = 0;
  virtual double evaluate(const Point& p) const = 0;
};

class UnionSDF : public SDFBase {
 public:
  using Ptr = std::shared_ptr<UnionSDF>;
  UnionSDF(std::vector<SDFBase::Ptr> sdfs) : sdfs_(sdfs) {}
  Values evaluate_batch(const Points& p) const override {
    Values vals = sdfs_[0]->evaluate_batch(p);
    for (size_t i = 1; i < sdfs_.size(); i++) {
      vals = vals.cwiseMin(sdfs_[i]->evaluate_batch(p));
    }
    return vals;
  }

  double evaluate(const Point& p) const override {
    double val = std::numeric_limits<double>::max();
    for (const auto& sdf : sdfs_) {
      val = std::min(val, sdf->evaluate(p));
    }
    return val;
  }

 private:
  std::vector<std::shared_ptr<SDFBase>> sdfs_;
};

class PrimitiveSDFBase : public SDFBase {
 public:
  using Ptr = std::shared_ptr<PrimitiveSDFBase>;
  PrimitiveSDFBase(const Pose& tf) : tf_(tf) {}

  Values evaluate_batch(const Points& p) const override {
    auto p_local = tf_.transform_points(p);
    return evaluate_in_local_frame(p_local);
  }

  double evaluate(const Point& p) const override {
    auto p_local = tf_.transform_point(p);
    return evaluate_in_local_frame(p_local);
  }

  Pose tf_;

 protected:
  virtual Values evaluate_in_local_frame(const Points& p) const = 0;
  virtual double evaluate_in_local_frame(const Point& p) const = 0;
};

class BoxSDF : public PrimitiveSDFBase {
 public:
  using Ptr = std::shared_ptr<BoxSDF>;
  Eigen::Vector3d width_;

  BoxSDF(const Eigen::Vector3d& width, const Pose& tf)
      : PrimitiveSDFBase(tf), width_(width) {}

 private:
  Values evaluate_in_local_frame(const Points& p) const override {
    auto&& half_width = width_ * 0.5;
    auto d = p.cwiseAbs().colwise() - half_width;
    auto outside_distance = (d.cwiseMax(0.0)).colwise().norm();
    auto inside_distance = d.cwiseMin(0.0).colwise().maxCoeff();
    Values vals = outside_distance + inside_distance;
    return vals;
  }

  double evaluate_in_local_frame(const Point& p) const override {
    auto&& half_width = width_ * 0.5;
    auto d = p.cwiseAbs() - half_width;
    auto outside_distance = (d.cwiseMax(0.0)).norm();
    auto inside_distance = d.cwiseMin(0.0).maxCoeff();
    return outside_distance + inside_distance;
  }
};

class CylinderSDF : public PrimitiveSDFBase {
 public:
  using Ptr = std::shared_ptr<CylinderSDF>;
  double radius_;
  double height_;
  CylinderSDF(double radius, double height, const Pose& tf)
      : PrimitiveSDFBase(tf), radius_(radius), height_(height) {}

 private:
  Values evaluate_in_local_frame(const Points& p) const override {
    Eigen::VectorXd&& d = p.topRows(2).colwise().norm();
    Eigen::Matrix2Xd p_projected(2, d.size());
    p_projected.row(0) = d;
    p_projected.row(1) = p.row(2);

    auto&& half_width = Eigen::Vector2d(radius_, height_ * 0.5);
    auto d_2d = p_projected.cwiseAbs().colwise() - half_width;
    auto outside_distance = (d_2d.cwiseMax(0.0)).colwise().norm();
    auto inside_distance = d_2d.cwiseMin(0.0).colwise().maxCoeff();
    Values vals = outside_distance + inside_distance;
    return vals;
  }

  double evaluate_in_local_frame(const Point& p) const override {
    double d = p.topRows(2).norm();
    Eigen::Vector2d p_projected(d, p(2));

    auto&& half_width = Eigen::Vector2d(radius_, height_ * 0.5);
    auto d_2d = p_projected.cwiseAbs() - half_width;
    auto outside_distance = (d_2d.cwiseMax(0.0)).norm();
    auto inside_distance = d_2d.cwiseMin(0.0).maxCoeff();
    return outside_distance + inside_distance;
  }
};

class SphereSDF : public PrimitiveSDFBase {
 public:
  using Ptr = std::shared_ptr<SphereSDF>;
  double radius_;

  SphereSDF(double radius, const Pose& tf)
      : PrimitiveSDFBase(tf), radius_(radius) {}

 private:
  Values evaluate_in_local_frame(const Eigen::Matrix3Xd& p) const override {
    return (p.colwise().norm().array() - radius_);
  }

  double evaluate_in_local_frame(const Point& p) const override {
    return (p.norm() - radius_);
  }
};

}  // namespace primitive_sdf
