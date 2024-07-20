find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS_INIT} -Wall -fPIC")

option(BUILD_TESTING "build google test" OFF)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(Threads REQUIRED)
  add_executable(unittest unittest.cpp)
  target_link_libraries(unittest ${GTEST_LIBRARIES} pthread)
  target_include_directories(unittest PUBLIC ${GTEST_INCLUDE_DIRS})
  enable_testing()
  gtest_discover_tests(unittest)
endif(BUILD_TESTING)

option(BUILD_BINDING "build python binding" OFF)
if(BUILD_BINDING)
  find_package(Python
    REQUIRED COMPONENTS Interpreter Development.Module)
  set(Python3_EXECUTABLE /usr/bin/python3.8)
  add_subdirectory(nanobind)

  nanobind_add_module(
    primitive_sdf
    STABLE_ABI
    NB_STATIC
    binding.cpp
  )
  # install(TARGETS nanobind_example_ext LIBRARY DESTINATION nanobind_example)
endif(BUILD_BINDING)
