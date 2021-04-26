cmake_minimum_required(VERSION 3.5)
project(behavior_tree)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(example_interfaces REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
  std_msgs
  example_interfaces
  )

add_executable(bt_engine src/BtEngine.cpp)
ament_target_dependencies(bt_engine ${dependencies})

add_library(fibonacci_bt_node SHARED plugins/action/fibonacci.cpp)
list(APPEND plugin_libs fibonacci_bt_node)

add_library(fibonacci_service_bt_node SHARED plugins/action/fibonacci_service.cpp)
list(APPEND plugin_libs fibonacci_service_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS bt_engine
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch trees
        DESTINATION share/${PROJECT_NAME}
        )

install (PROGRAMS
        scripts/fibonacci_action_server.py
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})

ament_package()
