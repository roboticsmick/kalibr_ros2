cmake_minimum_required(VERSION 3.22)
project(aslam_backend_python)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

find_package(ament_cmake REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS thread python)

include_directories(
  include
  ${numpy_eigen_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

# This function takes TARGET_NAME PYTHON_MODULE_DIRECTORY sourceFile1 [sourceFile2 ...]
add_python_export_library(${PROJECT_NAME} python/aslam_backend
  src/module.cpp
  src/Backend.cpp
  src/Optimizer.cpp
  src/OptimizerOptions.cpp
  src/OptimizationProblem.cpp
  src/DesignVariable.cpp
  src/ErrorTerm.cpp
  src/MEstimators.cpp
  src/JacobianContainer.cpp
  src/BackendExpressions.cpp
  src/CompressedColumnMatrix.cpp
  src/LinearSystemSolver.cpp
  src/ErrorTermTransformation.cpp
  src/TrustRegionPolicies.cpp
  src/SparseBlockMatrix.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  aslam_backend::aslam_backend
  aslam_backend_expressions::aslam_backend_expressions
  sparse_block_matrix::sparse_block_matrix
  sm_kinematics::sm_kinematics
  sm_timing::sm_timing
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
