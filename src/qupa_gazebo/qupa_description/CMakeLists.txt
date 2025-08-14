cmake_minimum_required(VERSION 2.8.3)

project(qupa_description)

find_package(catkin REQUIRED COMPONENTS
	roscpp 
	gazebo_ros
	qupa_msgs)

catkin_package(
	CATKIN_DEPENDS qupa_msgs message_runtime
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
