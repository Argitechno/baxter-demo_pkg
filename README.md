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
> #### `scripts/tests/add_two_ints_clients.py`
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
> #### `scripts/tests/add_two_ints_server.py`
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
> #### `scripts/tests/first_publish_node.py`
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
> #### `scripts/tests/first_subscriber_node.py`
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

> ``  
> #### `scripts/demos/acc_demo.py`
> **Description:**  
> This script demonstrates accessing and monitoring Baxter’s right arm accelerometer data. It initializes the accelerometer interface, then reads and prints the linear acceleration values twice a second for 5 seconds.  
>
> **Key Features:**  
> - Uses `baxter_interface` to check and manage the robot state.  
> - Interfaces with the accelerometer using a helper module `accelerometer_io`.  
> - Prints linear acceleration values in real time at 2 Hz.  
> - Includes a clean shutdown handler that disables the robot if it was initially disabled.  
> - Uses `argparse` to handle command-line arguments cleanly.  
>
> **Example Usage:**  
> ```bash
> rosrun demo_pkg acc_demo.py
> ```
>
> **Notes:**  
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.  
> The `demo_pkg.accelerometer_io` module must be available in the ROS package.
> Useful for debugging and validating accelerometer sensor output.
>
> ``

> ``
> #### `scripts/demos/camera_demo.py`
> **Description:**  
> This script demonstrates how to stream live camera output from Baxter’s head camera using OpenCV. It manages camera state to ensure only one hand camera is open at a time (due to hardware limits (max 2 of 3 cameras open)) and uses `cv_bridge` to convert ROS images for OpenCV display.
> 
> **Key Features:**  
> - Initializes the `head_camera` with resolution `(640, 400)` while closing another camera if needed.  
> - Uses `cv_bridge` to convert incoming ROS `Image` messages to OpenCV format.  
> - Subscribes to `/cameras/head_camera/image` and displays live video feed.  
> - Handles robot enable/disable status cleanly during shutdown.  
> - Uses `rospy.Rate` to limit display update frequency (and should allow quitting via 'q' key, but testing says otherwise).
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg camera_demo.py
> ```
> 
> **Notes:**  
> Requires OpenCV (`cv2`) and `cv_bridge`
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> Only one hand camera (left or right) can be open at a time on Baxter; the script enforces this.
> 
> ``

> ``  
> #### `scripts/demos/gripper_demo.py`
> **Description:**  
> Demonstrates basic control of Baxter’s right gripper. It calibrates the gripper, displays the current pose of the right wrist, and commands the gripper through a sequence of open/close positions.
> 
> **Key Features:**  
> - Initializes Baxter's right gripper and limb interfaces.  
> - Prints the current pose of the right end-effector (`endpoint_pose`).  
> - Calibrates and closes the gripper, then steps through positions: 100%, 75%, 50%, 25%, and 0%.  
> - Opens the gripper fully at the end of the sequence.  
> - Handles robot enable/disable state and cleans up on shutdown.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg gripper_demo.py
> ```
> 
> **Notes:**  
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> Ensure the gripper is attached and functioning.  
> Useful for verifying gripper calibration and testing response to position commands.
>
> ``

> ``  
> #### `scripts/demos/head_demo.py`
> **Description:**  
> Demonstrates basic control of Baxter’s head. It pans the head left and right, then centers it and performs a nodding motion. Useful for diagnostics, demonstrations, or adding personality to Baxter interactions.
> 
> **Key Features:**  
> - Accesses and commands the `head` interface.  
> - Pans the head right, then left, then centers it again.  
> - Executes a `nod` gesture.
> - Uses sleep intervals to pace head movement and gestures.  
> - Handles robot enable/disable state and clean shutdown behavior.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg head_demo.py
> ```
> 
> **Notes:**  
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> Head motion is limited to a small range; ensure nothing obstructs the head.
>
> ``

> ``  
> #### `scripts/demos/ik_demo.py`
> **Description:**  
> Demonstrates the use of Baxter’s Inverse Kinematics (IK) service to move the robot’s left arm to specific Cartesian poses using quaternion orientations. This script highlights how to query the IK solver, check the validity of the returned solution, and move the limb accordingly.
> 
> **Key Features:**  
> - Defines a utility `get_pose()` to create `PoseStamped` messages from position and quaternion arrays.  
> - Sends pose requests to Baxter’s `SolvePositionIK` ROS service.  
> - Validates IK results before commanding movement.  
> - Visits all 8 corners of a small cube centered at a base position to demonstrate arm reachability.  
> - Includes explanatory comments about quaternion effects on wrist/gripper orientation.  
> - Moves the arm to a center pose in a flipped (upside-down) orientation at the end.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg ik_demo.py
> ```
> 
> **Notes:**
> The robot must be powered on and enabled.  
> Uses only the left limb (`baxter_interface.Limb('left')`).  
> Depends on `numpy`
> Be aware of joint limits and workspace constraints when testing different poses.
>  
> ``

> ``  
> #### `scripts/demos/infrared_demo.py`
> **Description:**  
> Demonstrates how to access and read data from Baxter’s infrared range sensor using the `AnalogIO` interface. Specifically reads values from the right hand’s range sensor (`right_hand_range`) and prints the analog state over a 5-second period.
> 
> **Key Features:**  
> - Initializes an `AnalogIO` object for Baxter's right hand infrared sensor.  
> - Polls the sensor at 2 Hz and prints out its state (typically a distance measurement).  
> - Runs the polling loop for 5 seconds using `rospy.Time` for timing control.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg infrared_demo.py
> ```
> 
> **Notes:**  
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> The `right_hand_range` sensor topic must be connected and functional.  
> This script is useful for debugging or logging simple proximity information from Baxter's infrared sensor.
>
> ``

> ``  
> #### `scripts/demos/joint_demo.py`
> **Description:**  
> Demonstrates direct control of Baxter’s joints using the `baxter_interface.Limb` class. Moves both arms through a series of predefined joint configurations to test motion and verify joint accuracy.
> 
> **Key Features:**  
> - Initializes both left and right limb interfaces.  
> - Commands each limb to move to a series of hardcoded joint positions using radians.  
> - Includes `move_to_neutral()` to reset limbs to Baxter’s default neutral pose.  
> - Uses `numpy` for concise angle definitions in terms of π (pi).
> 
> **Joint Sequence Overview:**  
> 1. Both arms move to mirrored poses.  
> 2. Right arm performs an additional pose change.  
> 3. Both arms reset to neutral.  
> 4. Both arms execute a final mirrored pose.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg joint_demo.py
> ```
> 
> **Notes:**  
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> Robot should be in an uncrowded space to safely perform joint movements.
> It is a co-bot, but precautions are good.
> Useful for testing joint actuation and ensuring safe range of motion before running more complex tasks.
> Always keep the physical emergency stop accessible while testing custom joint positions to avoid potential collisions or joint limits being exceeded.
> 
> ``

> ``  
> #### `scripts/demos/lights_demo.py`
> **Description:**  
> Demonstrates controlling Baxter’s digital and sonar lights using ROS and the `baxter_interface.digital_io` along with a custom `SonarLightsIO` interface. Toggles various robot lights on/off, modulates sonar light intensity with trigonometric wave patterns, and runs a blinking light show sequence.
> 
> **Key Features:**  
> - Initializes digital I/O objects for multiple Baxter lights on arms, torso, and head.  
> - Implements a helper function to toggle light states.  
> - Cycles through light toggling with timed delays.  
> - Dynamically sets sonar light red and green intensities using cosine and sine waves.  
> - Executes a 5-second alternating binary blinking light show at 100 Hz.  
> - Uses ROS timing utilities (`rospy.Rate` and `rospy.Duration`) for precise control flow.
> 
> **Light Control Sequence Overview:**  
> 1. Toggle all main lights on/off sequentially with 1-second pauses.  
> 2. Gradually modulate red and green sonar light intensities over 12 steps.  
> 3. Perform a rapid blinking pattern alternating binary light masks for 5 seconds.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg lights_demo.py
> ```
> **Notes:**
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> Useful for testing and demonstrating Baxter’s light control capabilities.
>  
> ``

> ``  
> #### `scripts/demos/screen_demo.py`
> **Description:**  
> Demonstrates how to play a video on Baxter’s face screen using OpenCV and ROS. Publishes a frame-by-frame video stream to the `/robot/xdisplay` topic, followed by a static image display. Useful for showing custom animations or feedback on Baxter’s face display.
> 
> **Key Features:**  
> - Initializes a ROS node and publisher for Baxter’s screen (`/robot/xdisplay`).  
> - Loads and plays a video from the local assets folder.  
> - Scales video frames to fit Baxter’s screen (max 600px tall by 1024px wide).  
> - Publishes each video frame to the robot screen at the video’s native FPS.  
> - After the video ends (or after 20 seconds), displays a static fallback image.  
> - Uses OpenCV for video handling and image resizing, and `cv_bridge` to convert frames to ROS image messages.
> 
> **Display Sequence Overview:**  
> 1. Load and play a local video file frame-by-frame on Baxter’s face.  
> 2. After 20 seconds or ROS shutdown, fallback to displaying a single image.  
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg screen_demo.py
> ```
> 
> **Notes:**
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> The video (rickroll.mp4) and image (evil_baxter.jpeg) must be placed in the ../../assets/ directory relative to the script path.
> Requires OpenCV (`cv2`) and `cv_bridge`
> 
> ``

> ``  
> #### `scripts/demos/sonar_demo.py`
> **Description:**  
> Demonstrates how to enable and test Baxter's sonar sensors using the `SonarIO` class. Sequentially activates each of Baxter’s 12 sonar emitters to verify functionality and prints their active state.
> 
> **Key Features:**  
> - Initializes a ROS node and sonar interface using `demo_pkg.sonar_io.SonarIO`.  
> - Enables one sonar sensor at a time in a binary sequence using `set_sonars(2**i)`.  
> - Prints the number of active sonar sensors and their binary mask each second.  
> - After all sensors are tested, re-enables all sonars using a full-bitmask (`4095` = `0b111111111111`).
> 
> **Sonar Test Sequence:**  
> 1. Activates each sonar sensor individually in order.  
> 2. Waits 1 second between each activation for visibility.  
> 3. Ends by enabling all sonars.
> 
> **Example Usage:**  
> ```bash
> rosrun demo_pkg sonar_demo.py
> ```
> **Notes:**
> Requires Baxter to be powered on and ROS Kinetic properly configured & connected to robot.
> The test runs for 12 seconds and cycles through all sonar channels.
> 
> ``