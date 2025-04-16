cmake_minimum_required(VERSION 3.14)
project(lanelet2_extension)

# Autoware CMake 설정
find_package(autoware_cmake REQUIRED)
autoware_package()

# GeographicLib와 Pugixml 찾기
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

# 설치 디렉토리를 최우선으로 설정
set(CMAKE_PREFIX_PATH "/home/sws/lanelet2_ws/install" ${CMAKE_PREFIX_PATH})

# lanelet2 관련 패키지 찾기
find_package(lanelet2_core REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)
find_package(lanelet2_io REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)
find_package(lanelet2_maps REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)
find_package(lanelet2_projection REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)
find_package(lanelet2_routing REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)
find_package(lanelet2_traffic_rules REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)
find_package(lanelet2_validation REQUIRED PATHS "/home/sws/lanelet2_ws/install" NO_DEFAULT_PATH)

# 헤더 파일 포함 (find_package 변수 사용)
include_directories(
  ${lanelet2_core_INCLUDE_DIRS}
  ${lanelet2_io_INCLUDE_DIRS}
  ${lanelet2_maps_INCLUDE_DIRS}
  ${lanelet2_projection_INCLUDE_DIRS}
  ${lanelet2_routing_INCLUDE_DIRS}
  ${lanelet2_traffic_rules_INCLUDE_DIRS}
  ${lanelet2_validation_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
)

add_definitions(${GeographicLib_DEFINITIONS})

# 라이브러리 빌드
ament_auto_add_library(lanelet2_extension_lib SHARED
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/autoware_traffic_mirror.cpp # added | KMS_250313
  lib/crosswalk.cpp
  lib/detection_area.cpp
  lib/no_parking_area.cpp
  lib/no_stopping_area.cpp
  lib/message_conversion.cpp
  lib/mgrs_projector.cpp
  lib/query.cpp
  lib/road_marking.cpp
  lib/speed_bump.cpp
  lib/transverse_mercator_projector.cpp
  lib/utilities.cpp
  lib/virtual_traffic_light.cpp
  lib/visualization.cpp
  lib/route_checker.cpp
)

# 라이브러리 링크 (find_package 변수 사용)
target_link_libraries(lanelet2_extension_lib
  ${GeographicLib_LIBRARIES}
  ${PUGIXML_LIBRARIES}
  ${lanelet2_core_LIBRARIES}
  ${lanelet2_io_LIBRARIES}
  ${lanelet2_maps_LIBRARIES}
  ${lanelet2_projection_LIBRARIES}
  ${lanelet2_routing_LIBRARIES}
  ${lanelet2_traffic_rules_LIBRARIES}
  ${lanelet2_validation_LIBRARIES}
)

# 실행 파일 빌드
ament_auto_add_executable(lanelet2_extension_sample src/sample_code.cpp)
add_dependencies(lanelet2_extension_sample lanelet2_extension_lib)
target_link_libraries(lanelet2_extension_sample
  lanelet2_extension_lib
)

ament_auto_add_executable(autoware_lanelet2_validation src/validation.cpp)
add_dependencies(autoware_lanelet2_validation lanelet2_extension_lib)
target_link_libraries(autoware_lanelet2_validation
  ${catkin_LIBRARIES}
  ${PUGIXML_LIBRARIES}
  lanelet2_extension_lib
)

ament_auto_add_executable(check_right_of_way src/check_right_of_way.cpp)
add_dependencies(check_right_of_way lanelet2_extension_lib)
target_link_libraries(check_right_of_way
  ${catkin_LIBRARIES}
  ${PUGIXML_LIBRARIES}
  lanelet2_extension_lib
)

# 테스트 설정
if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(message_conversion-test test/src/test_message_conversion.cpp)
  target_link_libraries(message_conversion-test lanelet2_extension_lib)
  
  ament_add_ros_isolated_gtest(projector-test test/src/test_projector.cpp)
  target_link_libraries(projector-test lanelet2_extension_lib)
  
  ament_add_ros_isolated_gtest(query-test test/src/test_query.cpp)
  target_link_libraries(query-test lanelet2_extension_lib)
  
  ament_add_ros_isolated_gtest(regulatory_elements-test test/src/test_regulatory_elements.cpp)
  target_link_libraries(regulatory_elements-test lanelet2_extension_lib)
  
  ament_add_ros_isolated_gtest(utilities-test test/src/test_utilities.cpp)
  target_link_libraries(utilities-test lanelet2_extension_lib)
  target_include_directories(utilities-test SYSTEM PRIVATE
    ${lanelet2_core_INCLUDE_DIRS}
  )
  
  ament_add_ros_isolated_gtest(route-test test/src/test_route_checker.cpp)
  target_link_libraries(route-test lanelet2_extension_lib)
endif()

# 패키지 설정
set(CMAKE_PREFIX_PATH "/home/sws/lanelet2_ws/install" ${CMAKE_PREFIX_PATH})
ament_auto_package()