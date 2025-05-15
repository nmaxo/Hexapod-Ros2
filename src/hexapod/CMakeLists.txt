cmake_minimum_required(VERSION 3.8)
project(hexapod)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # Добавляем зависимость geometry_msgs

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegCommand.msg"
  "msg/LegStatus.msg"
  "msg/SensorData.msg"
  DEPENDENCIES geometry_msgs  # Указываем зависимости отдельно
)

ament_package()
