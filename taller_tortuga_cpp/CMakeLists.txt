cmake_minimum_required(VERSION 3.8)
project(taller_tortuga_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Servicio
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CambiarModo.srv"
  DEPENDENCIES std_msgs
)

# Ejecutable
add_executable(controlador src/controlador.cpp)

# DEPENDENCIAS COMPLETAS
ament_target_dependencies(controlador
  rclcpp
  rclcpp_components
  turtlesim
  geometry_msgs
  std_msgs
)

# LINK con la librería del servicio (¡ESTO ES CLAVE!)
target_link_libraries(controlador
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
)

# Forzar dependencia
add_dependencies(controlador ${PROJECT_NAME})

install(TARGETS controlador DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
ament_package()