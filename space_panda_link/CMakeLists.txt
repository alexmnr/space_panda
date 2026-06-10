cmake_minimum_required(VERSION 3.8)
project(space_panda_link)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  geometry_msgs
  moveit_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2
  tf2_ros
  Eigen3
)

# Find dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# Define the executable
add_executable(
  space_panda_link 
  src/space_panda_link.cpp 
  src/helper_functions.cpp 
  src/mimicing.cpp 
  src/wrench_passthrough.cpp 
  src/setup.cpp
)

# Add include directories for the executable
target_include_directories(space_panda_link PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the libraries
ament_target_dependencies(space_panda_link ${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS space_panda_link
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
