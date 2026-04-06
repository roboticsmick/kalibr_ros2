# Helpful function for adding Boost.Python extension modules for ROS2/ament_cmake.
# Usage:
#
# find_package(python_module REQUIRED)
# include(${python_module_DIR}/../../../share/python_module/cmake/add_python_export_library.cmake)
# add_python_export_library(${PROJECT_NAME}
#   ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME}
#   src/file1.cpp
#   src/file2.cpp
# )
#
# Arguments:
#   TARGET_NAME             - name of the shared library target
#   PYTHON_MODULE_DIRECTORY - path to the directory that contains __init__.py
#                             (the leaf directory name becomes the Python package name)
#   ARGN                    - source files

FUNCTION(add_python_export_library TARGET_NAME PYTHON_MODULE_DIRECTORY)

  # Derive the Python package name from the leaf of PYTHON_MODULE_DIRECTORY
  get_filename_component(TMP "${PYTHON_MODULE_DIRECTORY}/garbage.txt" PATH)
  get_filename_component(PYTHON_PACKAGE_NAME "${TMP}.txt" NAME_WE)

  # Find Python 3 (Interpreter + Development + NumPy)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

  # Find Boost.Python for Python 3.
  # Try the versioned component name first (Python 3.12 ships boost_python312),
  # then fall back to the generic python3 / python suffixes.
  find_package(Boost QUIET COMPONENTS python312)
  if(NOT Boost_PYTHON312_FOUND)
    find_package(Boost QUIET COMPONENTS python3)
    if(NOT Boost_PYTHON3_FOUND)
      find_package(Boost REQUIRED COMPONENTS python)
    endif()
  endif()

  # Expose Python, NumPy and Boost headers to the target
  include_directories(
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  # Build the extension module as a shared library
  add_library(${TARGET_NAME} SHARED ${ARGN})

  # Python extension modules must NOT carry a "lib" prefix
  set_target_properties(${TARGET_NAME} PROPERTIES PREFIX "")

  # Link against Python 3 runtime and Boost.Python
  target_link_libraries(${TARGET_NAME}
    ${Python3_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  # Determine the ament Python install directory.
  # ament_get_python_install_dir is available when ament_cmake_python is loaded;
  # fall back to the conventional path otherwise.
  if(COMMAND ament_get_python_install_dir)
    ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  else()
    set(PYTHON_INSTALL_DIR "lib/python3/dist-packages")
  endif()

  install(TARGETS ${TARGET_NAME}
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PYTHON_PACKAGE_NAME}"
    ARCHIVE DESTINATION "${PYTHON_INSTALL_DIR}/${PYTHON_PACKAGE_NAME}"
  )

  # Copy the .so into the build tree so that Python can import it without
  # a full install (mirrors what the old CATKIN_DEVEL_PREFIX copy did).
  set(BUILD_TREE_PYTHON_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/../python3/dist-packages/${PYTHON_PACKAGE_NAME}")
  add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${BUILD_TREE_PYTHON_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy
      "$<TARGET_FILE:${TARGET_NAME}>"
      "${BUILD_TREE_PYTHON_DIR}/$<TARGET_FILE_NAME:${TARGET_NAME}>"
    COMMENT "Copying ${TARGET_NAME} into build-tree Python directory"
  )

  # Register the build-tree copy for 'make clean'
  get_directory_property(AMCF ADDITIONAL_MAKE_CLEAN_FILES)
  list(APPEND AMCF
    "${BUILD_TREE_PYTHON_DIR}/$<TARGET_FILE_NAME:${TARGET_NAME}>")
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${AMCF}")

ENDFUNCTION()
