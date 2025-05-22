# ROS Kinetic Baxter Demo Package

## Overview

This package contains a variety of executable scripts that demonstrate usage of key features on the Baxter robot using ROS Kinetic. The primary goal is educational: to showcase how the baxter_interface provided by Rethink and ROS can be used together in practice.

---

## Directory Structure

### `assets/`

Contains media files used in demonstration scripts:
- Two images for `smile_and_wave.py`, displayed on Baxter's screen.
- One image and one video used by `screen_demo.py` (located in `scripts/demos/`).

### `msg/`

Defines a custom message type used in test scripts. This mock message demonstrates how to create and use custom ROS messages.

### `scripts/`

This is the main directory for executable scripts.

- **General Scripts**
    - `get_joint_pos.py`
        A minimal script that prints the robot's current joint positions to the console.

    - `smile_and_wave.py`
        Full demo where the robot displays a smile image and performs a waving motion.

- **demos/**  
  Contains scripts focusing on specific features of the Baxter robot:
  - `acc_demo.py`: Accelerometer demonstration.
  - `camera_demo.py`: Camera usage example.
  - `gripper_demo.py`: Gripper control.
  - `head_demo.py`: Head movement and nodding.
  - `ik_demo.py`: Inverse kinematics usage.
  - `infrared_demo.py`: IR sensor feedback.
  - `joint_demo.py`: Joint position and control.
  - `lights_demo.py`: LED/light control.
  - `screen_demo.py`: Displaying media on Baxter's screen.
  - `sonar_demo.py`: Sonar sensor demo.

- **tests/**  

    Basic ROS tests that don't interface with Baxter hardware. Useful for practicing core ROS concepts:
    - `add_two_ints_client.py` and `add_two_ints_server.py`: Example service client/server.
    - `first_publish_node.py`: Basic publisher node.
    - `first_subscriber_node.py`: Basic subscriber node.  



### `src/`

Contains supporting Python modules used across the scripts. These define helper classes for interacting with Baxter features not included in `baxter_interface`.

### `srv/`

Defines a custom service type used in the `add_two_ints` service test example. Demonstrates how to create and use ROS services.


## Script Details

> ``
> #### `scripts/get_joint_pos.py`
> **Description:**  
> A utility script that creates a ROS node (`get_joint_pos`) and prints the current joint angles of Baxter’s left and right arms to the terminal. It gives console feedback for each stage of execution and is useful for testing or debugging joint state readings.
> 
> **Key Features:**  
> - Uses `baxter_interface.Limb` to access joint angles.
> - Prints status updates to the console for better user awareness.
> - Requires Baxter to be powered on and ROS properly initialized.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg get_joint_pos.py
> ```
> **Notes:**
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to Robot.
>  
> ``

> ``
> #### `scripts/smile_and_wave.py`
> **Description:**  
> This demo script showcases Baxter performing a series of interactive motions combined with facial expression changes on its screen. The robot displays neutral and happy faces, moves its head and arms into specific poses, waves its right arm, opens/closes the gripper, nods, and flashes its lights. It illustrates how to combine `baxter_interface` motion commands with screen display and digital I/O controls.
>
> **Key Features:**  
> - Uses inverse kinematics service to calculate arm joint positions for waving and hip placement.  
> - Publishes images to Baxter’s display using OpenCV and `cv_bridge`.  
> - Demonstrates coordinated head, arm, gripper, and LED control sequences.  
> - Includes safety checks for IK solutions before moving.  
> - Shows how to manipulate Baxter’s digital I/O lights.
>
> **Example Usage:**  
> ```bash
> rosrun demo_pkg smile_and_wave.py
> ```
>
> **Notes:**  
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> Image assets must exist at ../assets/baxter_face_neutral.png and ../assets/baxter_face_happy.png relative to the script location.
> Ensure the Baxter’s IK service is available and functioning.  
>
> ``

> ``  
> #### `scripts/add_two_ints_clients.py`
> **Description:**  
> This script is a ROS service client that connects to the `add_two_ints` service, sends two integers, and prints the result. It is part of a basic ROS service demonstration used to illustrate service-client interaction in a ROS package.
> 
> **Key Features:**  
> - Waits for the `add_two_ints` service to become available.  
> - Sends two integer arguments provided via the command line.  
> - Receives and prints the sum returned by the service.  
> - Includes basic error handling for service call failures.  
> - Demonstrates use of `rospy.ServiceProxy` to create a client interface.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg add_two_clients.py 3 7
> ```
>
> **Notes:**  
>     Requires the add_two_ints service to be running, started by the corresponding service server script (add_two_ints_server.py).
>     The custom service message AddTwoInts.srv must be properly defined and built in the demo_pkg.
>     Must be executed in an environment where ROS is sourced and a roscore is running.
>
> ``

> ``  
> #### `scripts/add_two_ints_server.py`
> **Description:**  
> This script implements a basic ROS service server that listens for incoming requests on the `add_two_ints` service, performs integer addition, and returns the result. It is designed as part of a minimal ROS example to demonstrate how to define and use custom services.
> 
> **Key Features:**  
> - Initializes a ROS node named `add_two_ints_server`.  
> - Advertises the `add_two_ints` service using a custom service type.  
> - Defines a callback function that adds two integers and returns the result.  
> - Logs each request to the console.  
> - Uses `rospy.Service` and `rospy.spin()` to keep the node alive and responsive.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg add_two_ints_server.py
> ```
>
> **Notes:**  
> Must be paired with a client script (like `add_two_clients.py`) that calls the `add_two_ints` service.  
> The custom service message `AddTwoInts.srv` must be properly defined and built in the `demo_pkg`.  
> Requires a running `roscore` and a sourced ROS environment.
>
> ''


> ``  
> #### `scripts/first_publish_node.py`
> **Description:**  
> This script is a simple ROS publisher node that repeatedly publishes timestamped "hello world" messages to the `demo_chatter` topic. It demonstrates the basic structure of a ROS node that sends messages using `rospy.Publisher`.
> 
> **Key Features:**  
> - Initializes a ROS node named `demo_talker`.  
> - Creates a publisher that sends `std_msgs/String` messages on the `demo_chatter` topic.  
> - Publishes a new message at 10 Hz using `rospy.Rate`.  
> - Includes timestamp in the message content using `rospy.get_time()`.  
> - Logs each message to the console with `rospy.loginfo()`.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg first_publish_node.py
> ```
>
> **Notes:**  
>     Requires a running `roscore` and a sourced ROS environment.
>     Subscribers must listen on the `demo_chatter` topic with message type `std_msgs/String` to receive the published data.  
>     Useful for testing and debugging topic communication and message logging in ROS.
>
> ``

> ``  
> #### `scripts/first_subscriber_node.py`
> **Description:**  
> This script is a simple ROS subscriber node that listens to the `demo_chatter` topic. It demonstrates the basic structure of a ROS node that receives messages and triggers a callback using `rospy.Subscriber`.
> 
> **Key Features:**  
> - Initializes a ROS node named `demo_listener` with a unique name using `anonymous=True`.  
> - Creates a subscriber that receives `std_msgs/String` messages on the `demo_chatter` topic.  
> - Logs incoming messages using `rospy.loginfo` with the caller ID.  
> - Uses `rospy.spin()` to keep the node active and responsive.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg first_subscriber_node.py
> ```
>
> **Notes:**  
>     Requires a running `roscore` and a sourced ROS environment.  
>     The `demo_chatter` topic must be actively publishing `std_msgs/String` messages.  
>     Useful for testing topic communication and validating message flow in ROS.
>
> ``



