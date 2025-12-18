cmake_minimum_required(VERSION 3.8)
project(neupan_nav2_controller)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find Python development libraries with better compatibility
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(PYTHON python3)
endif()

# Fallback to system Python if pkg-config fails
if(NOT PYTHON_LIBRARIES)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
  set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
endif()

# Find NumPy include directory with fallback
execute_process(
  COMMAND python3 -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

# Fallback for NumPy include directory
if(NOT NUMPY_INCLUDE_DIR)
  execute_process(
    COMMAND python3 -c "import sys; import numpy; sys.stdout.write(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    ERROR_QUIET
  )
endif()

# Final check for NumPy
if(NOT NUMPY_INCLUDE_DIR)
  message(WARNING "Could not find NumPy include directory. You may need to install numpy: pip3 install numpy")
  set(NUMPY_INCLUDE_DIR "/usr/local/lib/python3.10/dist-packages/numpy/core/include")
endif()

message(STATUS "Python include dirs: ${PYTHON_INCLUDE_DIRS}")
message(STATUS "Python libraries: ${PYTHON_LIBRARIES}")
message(STATUS "NumPy include dir: ${NUMPY_INCLUDE_DIR}")

# Include directories
include_directories(include)

# Set dependencies list
set(dependencies
  rclcpp
  nav2_core
  nav2_common
  nav2_util
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# Create plugin library
add_library(${PROJECT_NAME} SHARED
  src/neupan_controller.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Link Python libraries
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE 
  ${PYTHON_INCLUDE_DIRS}
  ${NUMPY_INCLUDE_DIR}
)

# Plugin export
pluginlib_export_plugin_description_file(nav2_core neupan_controller_plugin.xml)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin configuration
install(FILES neupan_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

# Install scripts (if directory exists)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/scripts
    USE_SOURCE_PERMISSIONS
  )
endif()

# Install vendored NeuPAN resources (algorithm + configs + minimal models) - if exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NeuPAN-main)
  install(DIRECTORY vendor/NeuPAN-main/
    DESTINATION share/${PROJECT_NAME}/vendor/NeuPAN-main
  )
endif()

# Install in-package models directory - if exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/model)
  install(DIRECTORY model/
    DESTINATION share/${PROJECT_NAME}/model
  )
endif()

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add test directory
  add_subdirectory(test)
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package() 