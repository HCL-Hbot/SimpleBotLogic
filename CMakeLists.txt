cmake_minimum_required(VERSION 3.1...3.14)
project(simplebotlogic C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(audio_tools REQUIRED)
find_package(lowwi REQUIRED)

add_executable(simplebotlogic_node src/simplebotlogicnode.cpp)

ament_target_dependencies(simplebotlogic_node
   rclcpp
   std_msgs
   audio_tools
   lowwi
)
target_link_libraries(simplebotlogic_node "${cpp_typesupport_target}")

if(UNIX AND NOT APPLE)  # Linux only
 target_compile_options(simplebotlogic_node PRIVATE -Wall)
endif()

install(TARGETS
simplebotlogic_node
DESTINATION lib/${PROJECT_NAME}) 

ament_package()
