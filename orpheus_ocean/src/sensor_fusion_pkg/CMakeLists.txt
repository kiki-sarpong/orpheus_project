cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PROJECT_SIMFILE data_fusion_sim)
set(PROJECT_EXECNAME data_fusion)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(GTest REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(imu_tools)


include_directories(
  include
)

add_executable(${PROJECT_EXECNAME} 
src/main.cpp
src/IMUSubscriber.cpp
src/DepthSubscriber.cpp
src/IMUDepthFusion.cpp
src/VerticalVelocityPublisher.cpp
)

add_executable(${PROJECT_SIMFILE} scripts/PubSimScript.cpp)
ament_target_dependencies(${PROJECT_SIMFILE} rclcpp std_msgs sensor_msgs)

# First entry is the executable, the rest are dependencies
ament_target_dependencies(
${PROJECT_EXECNAME}
rclcpp
std_msgs
sensor_msgs
)

# Include executable targets
install(TARGETS
${PROJECT_EXECNAME} 
${PROJECT_SIMFILE}
DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(
 DIRECTORY include/
 DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
add_subdirectory(test)

ament_package()
