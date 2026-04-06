cmake_minimum_required(VERSION 3.22)
project(kalibr_imu_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(kalibr_common)

install(PROGRAMS
  scripts/kalibr_calibrate_imu_camera
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
