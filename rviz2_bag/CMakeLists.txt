cmake_minimum_required(VERSION 3.8)
project(rviz2_bag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

qt5_wrap_ui(UIC_FILES
  src/ui/player.ui
  src/ui/recorder.ui
)

add_library(rviz2_bag SHARED
  ${UIC_FILES}
  include/rviz2_bag/rosbag2_transport/player.hpp
  include/rviz2_bag/rosbag2_transport/recorder.hpp
  include/rviz2_bag/player.hpp
  include/rviz2_bag/recorder.hpp
  src/rosbag2_transport/logging.hpp
  src/rosbag2_transport/player.cpp
  src/rosbag2_transport/recorder.cpp
  src/rosbag2_transport/topic_filter.cpp
  src/rosbag2_transport/topic_filter.hpp
  src/player.cpp
  src/recorder.cpp
  src/icons/icons.qrc
)

target_compile_features(rviz2_bag PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rviz2_bag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
ament_target_dependencies(
  rviz2_bag
  "rclcpp"
  "rviz_common"
  "pluginlib"
  "rosbag2_interfaces"
  "rosbag2_storage"
  "rosbag2_transport"
  "Qt5"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz2_bag PUBLIC "RVIZ2_BAG_PLAYER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS rviz2_bag
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  rviz2_bag
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
