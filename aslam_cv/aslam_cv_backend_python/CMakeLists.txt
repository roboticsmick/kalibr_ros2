cmake_minimum_required(VERSION 3.22)
project(aslam_cv_backend_python)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

find_package(ament_cmake REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_cv_error_terms REQUIRED)
find_package(aslam_splines REQUIRED)
find_package(aslam_cv_python REQUIRED)
find_package(aslam_cv_backend REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_python REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(
  include
  ${numpy_eigen_INCLUDE_DIRS}
  ${sm_python_INCLUDE_DIRS}
  ${aslam_backend_python_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

add_python_export_library(${PROJECT_NAME} python/aslam_cv_backend
  src/module.cpp
  src/GridCalibration.cpp
  # src/CameraGeometryDesignVariableContainer.cpp
#  src/NCameraSystemDesignVariableContainer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  ${OpenCV_LIBS}
  aslam_backend::aslam_backend
  aslam_backend_expressions::aslam_backend_expressions
  aslam_cameras::aslam_cameras
  aslam_cv_backend::aslam_cv_backend
  aslam_cv_error_terms::aslam_cv_error_terms
  aslam_splines::aslam_splines
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
