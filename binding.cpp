#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include "primitive_sdf.hpp"

namespace nb = nanobind;
NB_MODULE(primitive_sdf, m) {
  m.doc() = "Primitive SDF module";
  nb::class_<primitive_sdf::Pose>(m, "Transform")
      .def(nb::init<const Eigen::Vector3d&, const Eigen::Matrix3d&>());
  nb::class_<primitive_sdf::BoxSDF>(m, "BoxSDF")
      .def(nb::init<const Eigen::Vector3d&, const primitive_sdf::Pose&>())
      .def("evaluate", &primitive_sdf::BoxSDF::evaluate);
  nb::class_<primitive_sdf::CylinderSDF>(m, "CylinderSDF")
      .def(nb::init<double, double, const primitive_sdf::Pose&>())
      .def("evaluate", &primitive_sdf::CylinderSDF::evaluate);
  nb::class_<primitive_sdf::SphereSDF>(m, "SphereSDF")
      .def(nb::init<double, const primitive_sdf::Pose&>())
      .def("evaluate", &primitive_sdf::SphereSDF::evaluate);
}
