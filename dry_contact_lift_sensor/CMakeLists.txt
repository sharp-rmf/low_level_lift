cmake_minimum_required(VERSION 3.5)
project(dry_contact_lift_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -lwiringPi -lcrypt -lpthread -lrt -lm")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(BCM2835 REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

include_directories(include)
include_directories(include /home/liftcontroller/bcm2835-1.58/src)
link_libraries(/home/liftcontroller/bcm2835-1.58/src/bcm2835.o)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    #"example_interfaces"
    "dry_contact_lift_node"
    "rclcpp"
    "rcutils"
    "std_msgs"
    "rmf_lift_msgs")
  target_link_libraries(${target} bcm2835)
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(. DryContactLiftWrap)

# function(custom_executable subfolder target)
#   add_executable(${target} src/${subfolder}/${target}.cpp)
#   ament_target_dependencies(${target}
#     "mock_atm_node"
#     "rclcpp"
#     "rcutils"
#     "rmf_msgs"
#     "std_msgs")
#   #target_link_libraries(${target} bcm2835)
#   install(TARGETS ${target}
#   DESTINATION lib/${PROJECT_NAME})
# endfunction()

# custom_executable(. Lift1TestApplication)

#include_directories(include ${WIRINGPI_INCLUDE_DIRS})

ament_package()
