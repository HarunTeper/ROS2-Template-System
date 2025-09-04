cmake_minimum_required(VERSION 3.8)
project(template_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Include directories
include_directories(include)

# Add component library
add_library(generic_node_component SHARED src/generic_node.cpp)

# Link dependencies
ament_target_dependencies(generic_node_component
  rclcpp
  rclcpp_components
  std_msgs
  ament_index_cpp
)

target_link_libraries(generic_node_component ${YAML_CPP_LIBRARIES})
target_include_directories(generic_node_component PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Register components
rclcpp_components_register_nodes(generic_node_component "template_system::GenericNode")

# Install targets
install(TARGETS
  generic_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
