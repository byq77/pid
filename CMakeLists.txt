cmake_minimum_required(VERSION 3.8)

project(pid)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(this_package_include_depends
  rclcpp
  rcutils
  std_msgs
  generate_parameter_library
)

foreach(dependency IN ITEMS ${this_package_include_depends})
    find_package(${dependency} REQUIRED)
endforeach()

# region pid_controller
set(target pid_controller)

generate_parameter_library(pid_controller_parameters
  src/pid/pid_controller.yaml
)

add_executable(${target}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/pid/pid_controller.cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/pid/pid.cpp>
)
target_include_directories(${target}
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${target}
                      PRIVATE
                      pid_controller_parameters
                      rclcpp::rclcpp
                      rcutils::rcutils
                      ${std_msgs_TARGETS})
install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
# endregion pid_controller

function(custom_executable subfolder target)
  add_executable(${target}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${subfolder}/${target}.cpp>
  )
  target_include_directories(${target}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(${target}
    "rclcpp"
    "rcutils"
    "std_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(pid servo_sim)
custom_executable(pid setpoint)

ament_export_include_directories(
  include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
