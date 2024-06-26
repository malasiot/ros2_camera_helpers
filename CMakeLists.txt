cmake_minimum_required(VERSION 3.8)
project(camera_helpers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED core imgproc)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(camera_gui
    src/camera_gui.cpp
    src/camera_gui.hpp
    src/image_widget.cpp
    src/image_widget.hpp
    src/images.qrc
    )
target_include_directories(camera_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(camera_gui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(camera_gui
    rclcpp
       sensor_msgs
       std_msgs
       cv_bridge
       message_filters
       image_geometry
)
target_link_libraries(camera_gui Qt::Core Qt::Gui Qt::Widgets ${OpenCV_LIBS})

install(TARGETS camera_gui
  DESTINATION lib/${PROJECT_NAME})
  
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
