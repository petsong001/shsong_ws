# Third Party Description 


### moveit2
* **Purpose:** Motion planning and collision avoidance. It handles the math and trajectory generation for the robot.
* **Source:** [https://github.com/ros-planning/moveit2](https://github.com/ros-planning/moveit2)
* **Setup:** `git clone -b humble https://github.com/ros-planning/moveit2.git`




### ros2_robotiq_gripper
* **Purpose:** Hardware drivers and 3D descriptions for the Robotiq Hand-E gripper. It allows ROS2 to physically open/close the gripper and draw it in RViz.
* **Source:** [https://github.com/PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
* **Setup:** `git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git`




### serial
* **Purpose:** A low-level C++ library for serial communication. It is used by other drivers to talk to hardware like microcontrollers or serial-based sensors.
* **Source:** [https://github.com/wjwwood/serial](https://github.com/wjwwood/serial) (or a ROS2 fork like [RoverRobotics-forks/serial-ros2](https://github.com/RoverRobotics-forks/serial-ros2))
* **Setup:** `git clone https://github.com/wjwwood/serial.git`




### trac_ik
* **Purpose:** An advanced Inverse Kinematics solver that replaces the default KDL solver. It provides much higher reliability and success rates when planning arm movements.
* **Source:** [https://github.com/traclabs/trac_ik](https://github.com/traclabs/trac_ik)
* **Setup:** `git clone -b humble https://github.com/traclabs/trac_ik.git`




### Universal Robots ROS2 Description
* **Purpose:** Contains the URDF (skeleton) and 3D meshes (skin) for the UR arm series. It is required for RViz visualization and collision checking in MoveIt.
* **Source:** [https://github.com/UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
* **Setup:** `git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git`




### Universal Robots ROS2 Driver
* **Purpose:** The hardware interface and communication bridge between ROS2 and the physical UR5e controller. It handles real-time control, calibration, and state feedback.
* **Source:** [https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
* **Setup:** `git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git`




### ur_description
* **Purpose:** The core ROS2 package containing the URDF models and 3D meshes for Universal Robots. It provides the physical dimensions used by MoveIt and RViz.
* **Source:** Part of the [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) repository.
* **Setup:** This is usually included automatically when you clone the Universal Robots description repository.