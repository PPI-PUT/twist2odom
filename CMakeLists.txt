cmake_minimum_required(VERSION 3.14)
project(twist_to_odom)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(TWIST_TO_ODOM_NODE_SRC
  src/twist_to_odom_node.cpp
)

set(TWIST_TO_ODOM_NODE_HEADERS
  include/twist_to_odom/twist_to_odom_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${TWIST_TO_ODOM_NODE_SRC}
  ${TWIST_TO_ODOM_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "twist_to_odom::TwistToOdomNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
