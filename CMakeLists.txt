cmake_minimum_required(VERSION 3.0.2)
project(demo_pkg)

find_package(catkin REQUIRED COMPONENTS

	rospy
	std_msgs
	message_generation
)


add_message_files(FILES

	Num.msg
)

add_service_files(FILES

	AddTwoInts.srv
)

## After add_message_files
## After add_service_files
## Before catkin_package
generate_messages(DEPENDENCIES
  std_msgs
)

## Build depends
catkin_package(CATKIN_DEPENDS

	rospy
	std_msgs
	message_runtime
)







## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
	scripts/first_publish_node.py 
	scripts/first_subscriber_node.py
	scripts/add_two_ints_server.py
	scripts/add_two_ints_client.py
	scripts/joint_demo.py
	scripts/head_demo.py
	scripts/gripper_demo.py
	scripts/accelerometer_listener.py
	scripts/ir_sensor_listener.py
	scripts/sonar_sensor_listener.py
	scripts/lights_talker.py
  	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

