cmake_minimum_required(VERSION 3.28.0)

project(aslam_cameras_april
  VERSION 0.0.1
  DESCRIPTION "ASLAM cameras Aprilgrid calibration target"
  LANGUAGES CXX
  )

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(sm_python REQUIRED)
find_package(aslam_cameras REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system python)
find_package(Eigen3 REQUIRED)
find_package(ethz_apriltag2 REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_logging REQUIRED)
find_package(sm_boost REQUIRED)
find_package(sm_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

# C++ library
add_library(${PROJECT_NAME}
  src/GridCalibrationTargetAprilgrid.cpp
  src/GridCalibrationTargetAprilgridv2.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  aslam_cameras::aslam_cameras
  Boost::serialization
  Boost::system
  Eigen3::Eigen
  ethz_apriltag2::ethz_apriltag2
  sm_common::sm_common
  sm_logging::sm_logging
  sm_boost::sm_boost
  sm_eigen::sm_eigen
  ${OpenCV_LIBS}
)

target_compile_options(
  ${PROJECT_NAME}
  PUBLIC
  "-Wall"
  "-Wextra"
)

target_compile_features(
  ${PROJECT_NAME}
  PUBLIC
  cxx_std_14
)

set(INSTALL_CMAKEDIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKEDIR})

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${INSTALL_CMAKEDIR})

# Python bindings
add_python_export_library(${PROJECT_NAME}_python python/aslam_cameras_april
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_python
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  ${sm_python_LIBRARIES}
  # Link against libaslam_cv_python.so so that camera geometry Boost.Serialization
  # type registrations are active when GridDetector observations are serialized.
  aslam_cameras::aslam_cameras
)

# Locate libaslam_cv_python.so and link it so Boost.Serialization camera
# geometry type registrations are active when GridDetector observations are serialized.
# CMAKE_INSTALL_PREFIX is package-specific in colcon; use the workspace root instead.
get_filename_component(_ws_install_dir "${CMAKE_INSTALL_PREFIX}/../" ABSOLUTE)
find_library(_aslam_cv_python_lib
  NAMES libaslam_cv_python.so
  HINTS
    "${_ws_install_dir}/aslam_cv_python/lib/python3.12/site-packages/aslam_cv"
  NO_DEFAULT_PATH
)
if(_aslam_cv_python_lib)
  message(STATUS "Found libaslam_cv_python.so: ${_aslam_cv_python_lib}")
  get_filename_component(_aslam_cv_python_dir "${_aslam_cv_python_lib}" DIRECTORY)
  target_link_libraries(${PROJECT_NAME}_python ${_aslam_cv_python_lib})
  set_target_properties(${PROJECT_NAME}_python PROPERTIES
    INSTALL_RPATH "${_aslam_cv_python_dir}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
else()
  message(WARNING "libaslam_cv_python.so not found — camera geometry serialization may fail at runtime")
endif()
target_include_directories(${PROJECT_NAME}_python PRIVATE
  include
  ${numpy_eigen_INCLUDE_DIRS}
  ${sm_python_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/aslam_cameras_april/
  DESTINATION "${PYTHON_INSTALL_DIR}/aslam_cameras_april"
  FILES_MATCHING PATTERN "*.py"
)

enable_testing()
add_subdirectory(test)

ament_package()
