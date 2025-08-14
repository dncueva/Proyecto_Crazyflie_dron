cmake_minimum_required(VERSION 3.0.2)
project(qupa_behavior)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  std_msgs
  qupa_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs qupa_msgs
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/qupa_monitoring.py
  scripts/tam_assign.py
  scripts/qupa_selective_strategy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(tam_seeker src/tam_seeker.cpp)
target_link_libraries(tam_seeker ${catkin_LIBRARIES})
add_dependencies(tam_seeker ${catkin_EXPORTED_TARGETS})

install(TARGETS tam_seeker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

