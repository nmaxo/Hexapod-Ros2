cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

# Стандарты компиляции
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

# Флаги компилятора
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Зависимости
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Для геометрических типов

# Генерация интерфейсов
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegCommand.msg"
  "msg/LegStatus.msg"
  "msg/SensorData.msg"
  "msg/MotorCmd.msg"
  "msg/Legfeedback.msg"
  DEPENDENCIES std_msgs geometry_msgs  # Обе зависимости
)

# Экспорт зависимостей
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)

# Линтеры
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Финальная настройка
ament_package()
