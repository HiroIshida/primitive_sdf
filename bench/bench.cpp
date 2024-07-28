#include "../primitive_sdf.hpp"
#include <Eigen/Core>
#include <memory>
#include <chrono>
#include <iostream>
using namespace primitive_sdf;

int main() {
  auto pose = Pose(Eigen::Vector3d(1, 1, 1), Eigen::Matrix3d::Identity());
  auto box = std::make_shared<BoxSDF>(Eigen::Vector3d(1, 1, 1), pose);
  auto cylinder = std::make_shared<CylinderSDF>(1.0, 1.0, pose);
  auto sdf = UnionSDF({box, cylinder});

  // create random 3 x 100 points
  Points p = Points::Random(3, 100);
  size_t n_iter = 100000;
  { 
    std::cout << "batch evaluateion for 100 points" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    for(size_t i = 0; i < n_iter; i++) {
      sdf.evaluate_batch(p);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time per iter: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / n_iter << " ns" << std::endl;
  }
  {
    std::cout << "iter evaluateion for 100 points" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    double value = 0;
    for(size_t i = 0; i < n_iter; i++) {
      for(size_t j = 0; j < p.cols(); j++) {
        value += sdf.evaluate(p.col(j));
      }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time per iter: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / n_iter << " ns" << std::endl;
    std::cout << value << std::endl; // dummy value to avoid compiler optimization
  }
}
