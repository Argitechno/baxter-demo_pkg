cmake_minimum_required(VERSION 3.0.2)
project(demo_pkg)

find_package(catkin REQUIRED COMPONENTS

	rospy
	std_msgs
	message_generation
	baxter_interface
)

catkin_python_setup()

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
	baxter_interface
)







## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
	scripts/tests/first_publish_node.py
	scripts/tests/first_subscriber_node.py
	scripts/tests/add_two_ints_server.py
	scripts/tests/add_two_ints_client.py

	scripts/demos/joint_demo.py
	scripts/demos/head_demo.py
	scripts/demos/gripper_demo.py
	scripts/demos/lights_demo.py
	scripts/demos/camera_demo.py
	scripts/demos/infrared_demo.py

	scripts/accelerometer_listener.py
	scripts/ir_sensor_listener.py
  	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

