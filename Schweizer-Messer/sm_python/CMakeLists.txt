cmake_minimum_required(VERSION 3.22)
project(sm_python)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_matrix_archive REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS system serialization python)

add_definitions(-D__STRICT_ANSI__)

include_directories(
  include
  ${numpy_eigen_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

# Set up the python exports.
SET(PY_PROJECT_NAME sm_python)
SET(PY_PACKAGE_DIR python/sm)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cpp
  src/export_rotational_kinematics.cpp
  src/export_rotations.cpp
  src/export_transformations.cpp
  src/export_quaternion_algebra.cpp
  src/export_homogeneous.cpp
  src/exportTransformation.cpp
  src/exportHomogeneousPoint.cpp
  src/exportTimestampCorrector.cpp
  src/exportPropertyTree.cpp
  src/exportEigen.cpp
  src/exportUncertainVector.cpp
  src/exportMatrixArchive.cpp
  src/export_kinematics_property_tree.cpp
  src/export_eigen_property_tree.cpp
  src/Logging.cpp
  src/exportNsecTime.cpp
  src/random.cpp
)

target_link_libraries(${PY_PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  sm_kinematics::sm_kinematics
  sm_timing::sm_timing
  sm_logging::sm_logging
  sm_property_tree::sm_property_tree
  sm_matrix_archive::sm_matrix_archive
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python source files alongside the compiled extension so that
# __init__.py, PlotCollection.py, etc. are available at import time.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/sm/
  DESTINATION "${PYTHON_INSTALL_DIR}/sm"
  FILES_MATCHING PATTERN "*.py"
)

ament_export_include_directories(include)
ament_package()
