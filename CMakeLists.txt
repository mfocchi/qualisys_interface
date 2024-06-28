cmake_minimum_required(VERSION 3.5)

project(qualisys)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake/modules")
set(QUALISYS_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/qualisys_cpp_sdk")

# Find dependencies
find_package(QUALISYSSDK REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Set dependencies
set(dependencies
  rclcpp
  geometry_msgs
  Eigen3
)

# Include directories
include_directories(
  include
  ${QUALISYSSDK_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC src/qualisys_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_library(qualisys_cpp_sdk STATIC IMPORTED)
set_target_properties(qualisys_cpp_sdk PROPERTIES
    IMPORTED_LOCATION ${QUALISYSSDK_LIBRARY})

# Add executable for qualisys_node
add_executable(qualisys_node src/qualisys_node.cpp)

# Link dependencies
ament_target_dependencies(qualisys_node ${dependencies})
target_link_libraries(qualisys_node
  ${PROJECT_NAME}
  qualisys_cpp_sdk
)

ament_target_dependencies(qualisys ${dependencies})
target_link_libraries(qualisys
  qualisys_cpp_sdk
)

# Install targets
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  qualisys_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
