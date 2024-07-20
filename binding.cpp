#include <pybind11/detail/common.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "primitive_sdf.hpp"
namespace py = pybind11;

namespace primitive_sdf {

PYBIND11_MODULE(primitive_sdf, m) {
  m.doc() = "Primitive SDF module";
  py::class_<Pose>(m, "Pose").def(
      py::init<const Eigen::Vector3d&, const Eigen::Matrix3d&>());
  py::class_<BoxSDF>(m, "BoxSDF")
      .def(py::init<const Eigen::Vector3d&, const Pose&>())
      .def("evaluate", &BoxSDF::evaluate);
  py::class_<CylinderSDF>(m, "CylinderSDF")
      .def(py::init<double, double, const Pose&>())
      .def("evaluate", &CylinderSDF::evaluate);
  py::class_<SphereSDF>(m, "SphereSDF")
      .def(py::init<double, const Pose&>())
      .def("evaluate", &SphereSDF::evaluate);
}

}  // namespace primitive_sdf
