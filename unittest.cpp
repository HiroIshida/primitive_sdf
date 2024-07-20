#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "primitive_sdf.hpp"

using namespace primitive_sdf;
using namespace Eigen;

TEST(TEST_BOXSDF, ALLTEST) {
  auto points = MatrixXd(8, 3);
  points << -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5,
      -0.5, -0.5, 0.5, 0.5, -0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
  Vector3d trans = Vector3d(0.5, 0.5, 0.5);
  {
    auto mat = Matrix3d::Identity();
    auto tf = Pose(Vector3d(0.5, 0.5, 0.5), mat);
    auto sdf = BoxSDF({1, 1, 1}, tf);
    auto values = sdf.evaluate(points.transpose().colwise() + trans);
    for (int i = 0; i < values.size(); i++) {
      ASSERT_NEAR(values(i), 0, 1e-6);
    }
  }
  {
    Matrix3d mat;
    mat << cos(M_PI / 4), -sin(M_PI / 4), 0, sin(M_PI / 4), cos(M_PI / 4), 0, 0,
        0, 1;
    auto tf = Pose(Vector3d(0.5, 0.5, 0.5), mat);
    auto sdf = BoxSDF({1, 1, 1}, tf);
    auto values = sdf.evaluate((mat * points.transpose()).colwise() + trans);
    for (int i = 0; i < values.size(); i++) {
      ASSERT_NEAR(values(i), 0, 1e-6);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
