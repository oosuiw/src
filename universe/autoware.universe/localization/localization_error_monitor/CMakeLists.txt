cmake_minimum_required(VERSION 3.14)
project(localization_error_monitor)

set(autoware_localization_msgs_DIR "/actual/path/to/autoware_localization_msgs/install/share/autoware_localization_msgs/cmake")  #KMS_250317


find_package(autoware_cmake REQUIRED)
find_package(autoware_localization_msgs REQUIRED)
autoware_package()

ament_auto_add_library(localization_error_monitor_module SHARED
  src/diagnostics.cpp
)

ament_target_dependencies(
  localization_error_monitor_module
  autoware_localization_msgs
)

ament_auto_add_executable(localization_error_monitor
  src/main.cpp
  src/node.cpp
)
target_link_libraries(localization_error_monitor localization_error_monitor_module)

if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gtest(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${test_name} localization_error_monitor_module)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()


  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_diagnostics.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
