#include <pybind11/detail/common.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "primitive_sdf.hpp"
namespace py = pybind11;

PYBIND11_MODULE(primitive_sdf, m) {
  m.doc() = "Primitive SDF module";
  py::class_<primitive_sdf::Pose>(m, "Pose").def(
      py::init<const Eigen::Vector3d&, const Eigen::Matrix3d&>());
  py::class_<primitive_sdf::BoxSDF>(m, "BoxSDF")
      .def(py::init<const Eigen::Vector3d&, const primitive_sdf::Pose&>())
      .def("evaluate", &primitive_sdf::BoxSDF::evaluate);
  py::class_<primitive_sdf::CylinderSDF>(m, "CylinderSDF")
      .def(py::init<double, double, const primitive_sdf::Pose&>())
      .def("evaluate", &primitive_sdf::CylinderSDF::evaluate);
  py::class_<primitive_sdf::SphereSDF>(m, "SphereSDF")
      .def(py::init<double, const primitive_sdf::Pose&>())
      .def("evaluate", &primitive_sdf::SphereSDF::evaluate);
}
