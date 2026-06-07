cmake_minimum_required(VERSION 3.8)
project(space_panda_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

######### Set Dependencies
set(DEPENDENCIES
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  franka_msgs
  Eigen3
  moveit_core
  moveit_ros_planning
  geometry_msgs
  realtime_tools
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})


######### Gravity Compensation Example Controller
add_library(
  ${PROJECT_NAME}
  SHARED
  src/space_panda_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
pluginlib_export_plugin_description_file(controller_interface space_panda_controller.xml)
install(
        TARGETS
        ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(
        include
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})
ament_package()
