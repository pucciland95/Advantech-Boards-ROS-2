cmake_minimum_required(VERSION 3.8)
project(advantech_usb4718)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces_advantech_usb4718 REQUIRED)

FIND_LIBRARY(advantech_LIBRARY libbiodaq.so /opt/advantech/libs)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    "/opt/advantech/inc"
)

add_executable(DAQ_Node src/DAQ_Node.cpp src/DAQ.cpp)
ament_target_dependencies(DAQ_Node rclcpp std_msgs interfaces_advantech_usb4718)
target_link_libraries(DAQ_Node ${advantech_LIBRARY})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  DAQ_Node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
