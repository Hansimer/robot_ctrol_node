cmake_minimum_required(VERSION 3.8)
project(robot_ctrol_node)

# 强制使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 若未指定构建类型，默认设为 Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -ggdb")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -ggdb")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_interfaces REQUIRED)
find_package(robot_controller REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_library(MODBUS_LIB modbus)
find_package(ethercat_control REQUIRED)
# 1. 指定头文件路径
include_directories(include
                    /home/nvidia/raybot_connection_ws/install/ethercat_control/include/
                    /home/nvidia/raybot_connection_ws/install/my_interfaces/include/
                    /usr/include/modbus
                    /home/nvidia/raybot_connection_ws/install/robot_controller/include/
                    )

# 2. 指定库路径和链接库
link_directories(/home/nvidia/raybot_connection_ws/install/ethercat_control/lib
                /home/nvidia/raybot_connection_ws/install/robot_controller/lib
)
# # 指定头文件路径
# include_directories(
#   include
#   /home/nvidia/raybot_connection_ws/install/my_interfaces/include/
#   /home/nvidia/raybot_connection_ws/install/robot_controller/include/
# )

# # 指定库路径
# link_directories(
#   /home/nvidia/raybot_connection_ws/install/robot_controller/lib
# )

# 编译可执行文件
add_executable(robot_ctrol_node
  src/robot_ctrol_node.cpp
  src/modbustcp_server.cpp
)

target_link_libraries(robot_ctrol_node 
${MODBUS_LIB}
tinyxml2
)


ament_target_dependencies(
  robot_ctrol_node
  rclcpp
  rclcpp_action
  my_interfaces
  sensor_msgs
  std_msgs
  robot_controller
  ethercat_control
  ament_index_cpp
)

# 安装可执行文件
install(TARGETS
  robot_ctrol_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 log 文件
install(DIRECTORY log
  DESTINATION log/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()