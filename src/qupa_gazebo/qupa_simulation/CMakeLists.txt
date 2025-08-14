cmake_minimum_required(VERSION 3.0.2)
project(qupa_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  geometry_msgs
  qupa_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES logical_camera_bridge
  CATKIN_DEPENDS roscpp gazebo_ros geometry_msgs
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${qupa_msgs_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Agregar la compilación del ejecutable
add_executable(logical_camera_bridge src/logical_camera_bridge.cpp)
add_executable(merge_odometry src/merge_odometry.cpp)
# Plugin TAM
add_library(TAMColorPlugin SHARED src/tam_plugin.cpp)

target_link_libraries(merge_odometry ${catkin_LIBRARIES} yaml-cpp)

target_link_libraries(logical_camera_bridge
  ${GAZEBO_LIBRARIES}
  ${GAZEBO_TRANSPORT_LIBRARIES}
  ${GAZEBO_COMMON_LIBRARIES}
  ${GAZEBO_MSGS_LIBRARIES}
  ${GAZEBO_RENDERING_LIBRARIES}
  ${GAZEBO_PHYSICS_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(TAMColorPlugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_dependencies(TAMColorPlugin ${catkin_EXPORTED_TARGETS})

# Asegurar que el nodo se instale correctamente
install(TARGETS logical_camera_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS TAMColorPlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


