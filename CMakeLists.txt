cmake_minimum_required(VERSION 3.20)
project(project1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable (run src/run.cpp)
target_link_libraries(run PUBLIC
behaviortree_cpp::behaviortree_cpp
px4_msgs::px4_msgs
nav_msgs::nav_msgs
behaviortree_ros2::behaviortree_ros2
ament_index_cpp::ament_index_cpp
)
install(TARGETS
  run
  DESTINATION lib/${PROJECT_NAME}
  )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY btcpp
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
