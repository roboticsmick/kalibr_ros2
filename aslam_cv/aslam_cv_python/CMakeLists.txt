cmake_minimum_required(VERSION 3.22)
project(aslam_cv_python)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(aslam_imgproc REQUIRED)
find_package(aslam_cv_serialization REQUIRED)
find_package(aslam_time REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(sm_python REQUIRED)
find_package(sm_opencv REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS serialization python)

if(APPLE)
  add_definitions(-ftemplate-depth-1024)
endif()

include_directories(
  include
  ${numpy_eigen_INCLUDE_DIRS}
  ${sm_python_INCLUDE_DIRS}
  ${aslam_imgproc_INCLUDE_DIRS}
  ${aslam_cv_serialization_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

add_python_export_library(${PROJECT_NAME} python/aslam_cv
  src/module.cpp
  src/CameraGeometries.cpp
  src/CameraGeometry.cpp
  src/Time.cpp
#  src/CameraSystem.cpp
#  src/CameraSystemClasses.cpp
#  src/MatchingAlgorithms.cpp
#  src/MultiFrame.cpp
  src/Frontend.cpp
  src/Frame.cpp
#  src/Descriptors.cpp
  src/CameraProjections.cpp
  src/CameraShutters.cpp
#  src/ImageSynchronizer.cpp
#  src/FrameBuilder.cpp
#  src/ImageContainer.cpp
  src/GridCalibration.cpp
#  src/Landmark.cpp
  src/Undistorters.cpp
#  src/NCameras.cpp
#  src/MultiFrameFactory.cpp
  src/OmniUndistorter.cpp
  src/PinholeUndistorter.cpp
)
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-deprecated-copy")
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  ${OpenCV_LIBS}
  aslam_cameras::aslam_cameras
  aslam_imgproc::aslam_imgproc
  aslam_cv_serialization::aslam_cv_serialization
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/aslam_cv/
  DESTINATION "${PYTHON_INSTALL_DIR}/aslam_cv"
  FILES_MATCHING PATTERN "*.py"
)

ament_package()