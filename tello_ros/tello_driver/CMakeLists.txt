cmake_minimum_required(VERSION 3.5)
project(tello_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ctello REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(move_base_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)

add_executable(tello_driver src/tello_driver.cpp src/state_publisher.cpp src/state.cpp src/camera_publisher.cpp src/movement_controller.cpp src/navigation.cpp src/astar_planner.cpp)
ament_target_dependencies(tello_driver rclcpp rclcpp_action sensor_msgs OpenCV ctello cv_bridge tf2_ros nav_msgs image_transport tello_msgs move_base_msgs octomap octomap_msgs dynamicEDT3D)
target_link_libraries(tello_driver ${OpenCV_LIBS} ctello ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})
target_include_directories(tello_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS tello_driver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
