cmake_minimum_required(VERSION 3.22)
project(kalibr_errorterms)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

find_package(ament_cmake REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(aslam_backend REQUIRED)
find_package(aslam_backend_expressions REQUIRED)
find_package(aslam_backend_python REQUIRED)
find_package(aslam_splines REQUIRED)
find_package(aslam_splines_python REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_common REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(
  include
  ${numpy_eigen_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

##################################
# C++ error terms library
##################################
add_library(${PROJECT_NAME}
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)

target_link_libraries(${PROJECT_NAME}
  aslam_backend::aslam_backend
  aslam_backend_expressions::aslam_backend_expressions
  aslam_splines::aslam_splines
  sm_kinematics::sm_kinematics
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

##################################
# Python bindings
##################################
# The ".." path places the .so one level up (in kalibr_errorterms_python/)
# so it can be imported as "import kalibr_errorterms_python" directly.
add_python_export_library(${PROJECT_NAME}_python python/${PROJECT_NAME}_python/..
  src/module.cpp
)

target_link_libraries(${PROJECT_NAME}_python
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
)

##################################
# Install
##################################
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
