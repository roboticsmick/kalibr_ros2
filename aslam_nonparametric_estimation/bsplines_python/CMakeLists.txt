cmake_minimum_required(VERSION 3.22)
project(bsplines_python)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-fPIC -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(python_module REQUIRED)
find_package(numpy_eigen REQUIRED)
find_package(bsplines REQUIRED)
find_package(sparse_block_matrix REQUIRED)
find_package(sm_common REQUIRED)
find_package(sm_kinematics REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(
  ${numpy_eigen_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)

add_python_export_library(${PROJECT_NAME} python/bsplines
  src/SplinePython.cpp
  src/BSplinePython.cpp
  src/BSplinePosePython.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${Python3_LIBRARIES}
  bsplines::bsplines
  sm_kinematics::sm_kinematics
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY python/bsplines/
  DESTINATION "${PYTHON_INSTALL_DIR}/bsplines"
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
