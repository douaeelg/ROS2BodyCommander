# ROS2BodyCommander

This project took part at LORIA Lab where the goal is to control robot motion via ros2 and pretrained ai models
The robot unitree go2 was used for testing .
# Human-Robot Interaction with Gesture Control

This repository contains the code for a ROS2-based robotic project that enables gesture-to-action execution and human tracking using a webcam. The system subscribes to human body tracking data, processes gestures, and sends commands to a robot (e.g., Unitree) via the `unitree_api`. It also publishes camera images and camera info for human detection.

## Repository Structure

```
├── commander_new.py        # Subscribes to tracked bodies, dynamically subscribes to gestures, and sends robot commands
├── listener.py             # Tracks the first detected human and rotates the robot to face them
├── pub_new_info.py         # Publishes webcam images and camera info
└── README.md               # This file
```

## Prerequisites

- **Python**: Version 3.10.12 or higher.
- **ROS2**: Jazzy distribution (Humble may also work).
- **Operating System**: Linux-based system (e.g., Ubuntu).
- A webcam connected to the system (default: `/dev/video0`).

## Installation

### 1. ROS2 Setup
Install ROS2 Jazzy distribution. Follow the official instructions for your Linux system (e.g., Ubuntu 24.04 for Jazzy). Ensure ROS2 is sourced in your terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Install Dependencies
Create a ROS2 workspace and install required libraries:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
pip install opencv-python numpy annonce
pip install lap ikpy
sudo apt install ros-jazzy-xacro
```

### 3. Install ROS2 Packages
Clone the necessary ROS2 packages into your workspace's `src` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros4hri/hri_body_detect.git
git clone https://github.com/ros4hri/hri_msgs.git
git clone https://github.com/ros2/rclpy.git
git clone https://github.com/pal-robotics/launch_pal.git
git clone https://github.com/ros4hri/human_description.git
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

### 4. Copy Project Files
Place the provided Python scripts (`commander_new.py`, `listener.py`, `pub_new_info.py`) into the `~/ros2_ws/src` directory.

### 5. Build the Workspace
Build the workspace using `colcon`:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 6. Configure Detection Confidence
Set the detection confidence threshold to 80% by editing `~/ros2_ws/src/hri_body_detect/config/00-defaults.yml`:

```yaml
detection_conf_thresh: 0.8
```

## Usage

1. **Launch Human Body Detection**:
   Open a terminal, source the workspace, and run:

   ```bash
   ros2 launch hri_body_detect hri_body_detect.launch.py
   ```

2. **Publish Webcam Data**:
   In a new terminal, source the workspace and run:

   ```bash
   python3 ~/ros2_ws/src/pub_new_info.py
   ```

3. **Activate Gesture-to-Action Control**:
   In another terminal, source the workspace and run:

   ```bash
   python3 ~/ros2_ws/src/commander_new.py
   ```

4. **Activate Human Tracking**:
   In a separate terminal, source the workspace and run:

   ```bash
   python3 ~/ros2_ws/src/listener.py
   ```

## Gesture-to-Action Mapping
The `commander_new.py` script maps gestures to robot actions with a 10-second debounce to avoid false positives:

- **Open Palm (ID 8)**: Sends "Hello" command (API ID 1016).
- **Thumb Down (ID 10)**: Sends "Sit" command (API ID 1005).
- **Thumb Up (ID 11)**: Sends "Stand" command (API ID 1004).
- **Victory (ID 12)**: Sends "Dance" command (API ID 1022).
- **Fist (ID 7)**: Sends "Stretch" command (API ID 1017).

## Human Tracking
The `listener.py` script tracks the first detected human and rotates the robot to face them using proportional control (gain: 0.7 for rotation, 0.6 for translation). The robot stops if the human is within ±7° of the camera's center or if no human is detected.

## Limitations
- **Gesture Detection**: The system may misinterpret waving as an open palm.
- **Hand-Torso Overlap**: Struggles when a person’s hand overlaps with or is close to their torso.
- **Webcam Dependency**: Ensure the webcam is correctly configured (modify `pub_new_info.py` if not `/dev/video0`).

## Notes
- Ensure the webcam is accessible and the correct device index is set in `pub_new_info.py`.
- The system assumes a Unitree robot with the `unitree_api` interface.
- All scripts must run in separate terminals with the ROS2 workspace sourced.# Human-Robot Interaction with Gesture Control

This repository contains the code for a ROS2-based robotic project that enables gesture-to-action execution and human tracking using a webcam. The system subscribes to human body tracking data, processes gestures, and sends commands to a robot (e.g., Unitree) via the `unitree_api`. It also publishes camera images and camera info for human detection.

## Repository Structure

```
├── commander_new.py        # Subscribes to tracked bodies, dynamically subscribes to gestures, and sends robot commands
├── listener.py             # Tracks the first detected human and rotates the robot to face them
├── pub_new_info.py         # Publishes webcam images and camera info
└── README.md               # This file
```

## Prerequisites

- **Python**: Version 3.10.12 or higher.
- **ROS2**: Jazzy distribution (Humble may also work).
- **Operating System**: Linux-based system (e.g., Ubuntu).
- A webcam connected to the system (default: `/dev/video0`).

## Installation

### 1. ROS2 Setup
Install ROS2 Jazzy distribution. Follow the official instructions for your Linux system (e.g., Ubuntu 24.04 for Jazzy). Ensure ROS2 is sourced in your terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Install Dependencies
Create a ROS2 workspace and install required libraries:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
pip install opencv-python numpy annonce
pip install lap ikpy
sudo apt install ros-jazzy-xacro
```

### 3. Install ROS2 Packages
Clone the necessary ROS2 packages into your workspace's `src` directory:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros4hri/hri_body_detect.git
git clone https://github.com/ros4hri/hri_msgs.git
git clone https://github.com/ros2/rclpy.git
git clone https://github.com/pal-robotics/launch_pal.git
git clone https://github.com/ros4hri/human_description.git
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

### 4. Copy Project Files
Place the provided Python scripts (`commander_new.py`, `listener.py`, `pub_new_info.py`) into the `~/ros2_ws/src` directory.

### 5. Build the Workspace
Build the workspace using `colcon`:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 6. Configure Detection Confidence
Set the detection confidence threshold to 80% by editing `~/ros2_ws/src/hri_body_detect/config/00-defaults.yml`:

```yaml
detection_conf_thresh: 0.8
```

## Usage

1. **Launch Human Body Detection**:
   Open a terminal, source the workspace, and run:

   ```bash
   ros2 launch hri_body_detect hri_body_detect.launch.py
   ```

2. **Publish Webcam Data**:
   In a new terminal, source the workspace and run:

   ```bash
   python3 ~/ros2_ws/src/pub_new_info.py
   ```

3. **Activate Gesture-to-Action Control**:
   In another terminal, source the workspace and run:

   ```bash
   python3 ~/ros2_ws/src/commander_new.py
   ```

4. **Activate Human Tracking**:
   In a separate terminal, source the workspace and run:

   ```bash
   python3 ~/ros2_ws/src/listener.py
   ```

## Gesture-to-Action Mapping
The `commander_new.py` script maps gestures to robot actions with a 10-second debounce to avoid false positives:

- **Open Palm (ID 8)**: Sends "Hello" command (API ID 1016).
- **Thumb Down (ID 10)**: Sends "Sit" command (API ID 1005).
- **Thumb Up (ID 11)**: Sends "Stand" command (API ID 1004).
- **Victory (ID 12)**: Sends "Dance" command (API ID 1022).
- **Fist (ID 7)**: Sends "Stretch" command (API ID 1017).

## Human Tracking
The `listener.py` script tracks the first detected human and rotates the robot to face them using proportional control (gain: 0.7 for rotation, 0.6 for translation). The robot stops if the human is within ±7° of the camera's center or if no human is detected.

## Limitations
- **Gesture Detection**: The system may misinterpret waving as an open palm.
- **Hand-Torso Overlap**: Struggles when a person’s hand overlaps with or is close to their torso.
- **Webcam Dependency**: Ensure the webcam is correctly configured (modify `pub_new_info.py` if not `/dev/video0`).

## Notes
- Ensure the webcam is accessible and the correct device index is set in `pub_new_info.py`.
- The system assumes a Unitree robot with the `unitree_api` interface.
- All scripts must run in separate terminals with the ROS2 workspace sourced.
