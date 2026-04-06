cmake_minimum_required(VERSION 3.22)
project(numpy_eigen)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(python_module REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(-D__STRICT_ANSI__)
add_compile_options(-fpermissive)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include/numpy_eigen"
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

include(src/autogen_files.cmake)
add_python_export_library(${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME}
  ${AUTOGEN_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
