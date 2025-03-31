# ROS2 Project - Robot Navigation and Color Detection

This project implements a robot that can navigate and detect RGB colored boxes. When a blue box is detected, the robot moves to within 1 meter of it and stops.

## Project Structure

```
src/ros2_project_xxx/
├── launch/
│   └── robot.launch.py
├── resource/
│   └── ros2_project_xxx
├── src/ros2_project_xxx/
│   ├── __init__.py
│   ├── robot_controller.py
│   └── color_detector.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Implementation Details

1. **Robot Controller**: Handles navigation and exploration of the environment. When a blue box is detected, it navigates to within 1 meter of the box and stops.

2. **Color Detector**: Processes camera images to detect red, green, and blue boxes. It publishes the detection results and a processed image with bounding boxes.

## How to Run

1. Install ROS2 (Humble or Foxy)

2. Clone the turtlebot3_simulations repository and switch to the project branch:
   ```bash
   git clone https://github.com/COMP3631-2025/turtlebot3_simulations.git
   cd turtlebot3_simulations
   git checkout 2025_ros_project
   ```

3. Build the turtlebot3_simulations package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select turtlebot3_simulations
   source install/setup.bash
   ```

4. Build the ros2_project_xxx package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2_project_xxx
   source install/setup.bash
   ```

5. Launch the simulation:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py
   ```

6. In a new terminal, launch the robot controller and color detector:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch ros2_project_xxx robot.launch.py
   ```

## Expected Behavior

- The robot will start exploring the environment.
- When it detects colored boxes, they will be highlighted in the processed image.
- When a blue box is detected, the robot will navigate to within 1 meter of it and stop.

## Video Demonstration

A video demonstration showing the robot performing the task is required for submission. The video should include:
- Task world gazebo simulation showing the robot performing the task
- Navigation in nav2 showing the robot path planning
- Window display of processed camera feed showing detection of RGB objects
