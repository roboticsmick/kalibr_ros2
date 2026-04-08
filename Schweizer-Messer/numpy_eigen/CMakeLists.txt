cmake_minimum_required(VERSION 3.22)
project(numpy_eigen)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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

# Install __init__.py so that 'import numpy_eigen' actually loads libnumpy_eigen
# and registers the numpy <-> Eigen matrix converters. Without this the package
# directory is empty and every C++ function that accepts an Eigen matrix will
# reject numpy array arguments.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES src/${PROJECT_NAME}/__init__.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

ament_export_include_directories(include)
ament_package()
