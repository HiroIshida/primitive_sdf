find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

option(BUILD_BINDING "build python binding" ON)
if(BUILD_BINDING)
  add_subdirectory(pybind11)
  pybind11_add_module(_psdf binding.cpp)
  install(TARGETS _psdf DESTINATION .)
endif(BUILD_BINDING)
