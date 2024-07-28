#include "../primitive_sdf.hpp"
#include <Eigen/Core>
#include <memory>
#include <chrono>
#include <iostream>
using namespace primitive_sdf;

int main() {
  auto pose = Pose(Eigen::Vector3d(0, 0, 0), Eigen::Matrix3d::Identity());
  auto box = std::make_shared<BoxSDF>(Eigen::Vector3d(1, 2, 1), pose);
  auto sphere = std::make_shared<SphereSDF>(0.8, pose);
  auto cylinder = std::make_shared<CylinderSDF>(1.0, 1.0, pose);
  auto sdf = UnionSDF({box, sphere, cylinder});

  // create random 3 x 10000 points
  Points p = Points::Random(3, 100);
  { 
    auto start = std::chrono::high_resolution_clock::now();
    for(size_t i = 0; i < 100000; i++) {
      sdf.evaluate_batch(p);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
  }
  {
    auto start = std::chrono::high_resolution_clock::now();
    for(size_t i = 0; i < 100000; i++) {
      for(size_t j = 0; j < p.cols(); j++) {
        sdf.evaluate(p.col(j));
      }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
  }
}
