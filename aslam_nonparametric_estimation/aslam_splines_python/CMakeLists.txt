cmake_minimum_required(VERSION 3.22)
project(aslam_splines_python)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(aslam_splines REQUIRED)
find_package(bsplines_python REQUIRED)
find_package(bsplines REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_timing REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(sm_property_tree REQUIRED)
find_package(sm_logging REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_time REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(
  ${numpy_eigen_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

add_python_export_library(${PROJECT_NAME} python/aslam_splines
  src/spline_module.cpp
  src/BSplineMotionError.cpp
  src/SimpleSplineError.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  aslam_splines::aslam_splines
  bsplines::bsplines
  aslam_backend::aslam_backend
  aslam_backend_expressions::aslam_backend_expressions
  aslam_cameras::aslam_cameras
  sm_kinematics::sm_kinematics
  sm_timing::sm_timing
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/aslam_splines/
  DESTINATION "${PYTHON_INSTALL_DIR}/aslam_splines"
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
