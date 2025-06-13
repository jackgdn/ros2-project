cmake_minimum_required(VERSION 3.8)
project(visual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
include_directories(
  include
  /opt/tros/include
  /usr/include
  /usr/include/zbar

)
pkg_check_modules(ZBAR REQUIRED zbar)
include_directories(${ZBAR_INCLUDE_DIRS})
link_directories(${ZBAR_LIBRARY_DIRS})
add_definitions(${ZBAR_CFLAGS_OTHER})

add_executable(qrcode src/Qrcode.cpp)
target_link_libraries(qrcode ${ZBAR_LIBRARIES})
ament_target_dependencies(qrcode rclcpp std_msgs geometry_msgs sensor_msgs cv_bridge OpenCV)
install(TARGETS qrcode DESTINATION lib/${PROJECT_NAME})


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

ament_package()
