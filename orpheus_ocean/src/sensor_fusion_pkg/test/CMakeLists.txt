# Set the name for the testproject
set(PROJECT_TESTFILE DataFusionTest)  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gtest(${PROJECT_TESTFILE} 
  DataFusionTest.cpp 
  ../src/IMUDepthFusion.cpp
  ../src/IMUSubscriber.cpp
  ../src/DepthSubscriber.cpp
  ../src/IMUDepthFusion.cpp
  ../src/VerticalVelocityPublisher.cpp
  )

  target_include_directories(${PROJECT_TESTFILE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_TESTFILE}
    std_msgs
    rclcpp
    sensor_msgs
  )

  target_link_libraries(${PROJECT_TESTFILE} GTest::gtest_main)

  ament_lint_auto_find_test_dependencies()

  # Automatic discovery of unit tests
    include(GoogleTest)
    gtest_discover_tests(${PROJECT_TESTFILE})
endif()