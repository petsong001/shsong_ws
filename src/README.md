# 🦾 UR5e + Hand-E: Modular Robotics Workspace

This repository contains a modular ROS 2 implementation for autonomous pick-and-place tasks. The architecture is designed for **Sim-to-Real** portability, allowing the same logic to control both a virtual robot in **NVIDIA Isaac Sim** and physical hardware in the lab.

---

## 📂 Package Map

| Package | Category | Primary Responsibility |
| :--- | :--- | :--- |
| **`my_robot_description`** | **Assets** | The "Single Source of Truth." Contains URDFs, meshes, and USD models. |
| **`ur5e_hande_bringup`** | **Config** | The system "Nervous System." Manages MoveIt configs, controller YAMLs, and master launchers. |
| **`ur5e_vision`** | **Perception** | The "Eyes." Handles ZED camera SDK (Real) and geometric projection (Sim). |
| **`ur5e_task_control`** | **Logic** | The "Brain." Orchestrates the pick-and-place state machine and gripper actions. |

---

## 📋 Package Details

### 1. `my_robot_description`
* **Purpose:** Defines the kinematic chain and visual geometry.
* **Key Files:** * `ur5e_hande.urdf.xacro`: The core robot model.
  * `view_ur5e.launch.py`: Debug tool to visualize the URDF in RViz.
  * `USD/`: High-fidelity assets for Isaac Sim.

### 2. `ur5e_hande_bringup` (Formerly `ur5e_hande_complete`)
* **Purpose:** Hardware abstraction layer and system initialization.
* **Key Files:**
  * `moveit_controllers_real.yaml` / `_sim.yaml`: Specialized PID and controller settings.
  * `isaac_sim_interface.launch.py`: Connection bridge for the simulator.
  * `real_moveit.launch.py`: Primary launcher for physical lab operations.

### 3. `ur5e_vision`
* **Purpose:** Translates visual data into actionable world coordinates.
* **Key Files:**
  * `zed_locator.py`: Uses ZED SDK for real-world depth and pose estimation.
  * `sim_locator_node.py`: Simplified locator for Isaac Sim environments.

### 4. `ur5e_task_control`
* **Purpose:** Executes high-level robotic tasks.
* **Key Files:**
  * `smart_pick_node.py`: Main autonomous sequence (Hover -> Dive -> Grasp -> Drop).
  * `gripper_control_node.py`: Independent action interface for the Hand-E gripper.