cmake_minimum_required(VERSION 3.5)

# Project name
project(uwb_ranger)

#################################################
# compiler settings
#################################################

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rclc REQUIRED)
find_package(rclc_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclc_parameter REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

# Source files
set(SOURCES
    src/rcmRange.c
    src/rcm.c
    src/rcmIf.c
)

# Source files
set(SOURCES2
    src/rcmRange2.c
    src/rcm.c
    src/rcmIf.c
)


# Executable target
add_executable(ranger ${SOURCES})
ament_target_dependencies(ranger rcl rclc std_msgs)

add_executable(ranger2 ${SOURCES2})
ament_target_dependencies(ranger2 rcl rclc std_msgs)

install(TARGETS
  ranger
  ranger2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
