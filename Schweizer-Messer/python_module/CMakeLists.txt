cmake_minimum_required(VERSION 3.22)
project(python_module)

find_package(ament_cmake REQUIRED)

install(FILES cmake/add_python_export_library.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(
  CONFIG_EXTRAS cmake/add_python_export_library.cmake
)
