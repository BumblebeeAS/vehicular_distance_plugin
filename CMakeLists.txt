cmake_minimum_required(VERSION 3.8)
project(vehicular_distance_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(Distance_SensorSystem SHARED
  src/Distance_SensorSystem.cc
)

target_compile_features(Distance_SensorSystem PUBLIC cxx_std_17)

target_include_directories(Distance_SensorSystem
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(Distance_SensorSystem
  PUBLIC
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-msgs10::gz-msgs10
    gz-plugin2::register
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    sdformat14::sdformat14
)

install(
  TARGETS Distance_SensorSystem
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  gz-common5
  gz-math7
  gz-msgs10
  gz-plugin2
  gz-sim8
  gz-transport13
  sdformat14
)
# Make libDistance_SensorSystem.so discoverable by Gazebo after sourcing the
# workspace installation.
set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_distance_sensor_plugin
  "prepend-non-duplicate;GZ_SIM_SYSTEM_PLUGIN_PATH;lib")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/distance_sensor_plugin.sh.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
