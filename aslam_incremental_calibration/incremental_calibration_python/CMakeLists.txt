cmake_minimum_required(VERSION 3.22)
project(incremental_calibration_python)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(incremental_calibration REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(
  ${numpy_eigen_INCLUDE_DIRS}
  ${aslam_backend_python_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

add_python_export_library(${PROJECT_NAME} python/incremental_calibration
  src/module.cpp
  src/OptimizationProblem.cpp
  src/IncrementalEstimator.cpp
  src/LinearSolver.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  incremental_calibration::incremental_calibration
  aslam_backend::aslam_backend
  sm_property_tree::sm_property_tree
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/incremental_calibration/
  DESTINATION "${PYTHON_INSTALL_DIR}/incremental_calibration"
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
