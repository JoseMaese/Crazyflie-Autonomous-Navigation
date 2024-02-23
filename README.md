# Quadrotor Autonomous Navigation System

This Python script controls a quadrotor in a simulated environment using the Webots robotics simulator. The key features of the script include:

- **Obstacle Avoidance:** The quadrotor stops before colliding with obstacles in all directions.
- **Predefined Path:** The script follows a circular predefined path.
- **Object Recognition:** Utilizes a YOLO neural network model for real-time object recognition.

## Authors

- [@josemaese](https://github.com/JoseMaese)
- [@santiagomoreno0123](https://github.com/santiagomoreno0123)

## Dependencies

- `controller`, `Keyboard`, and `Supervisor` modules are imported from Webots.
- Various controllers such as PID controllers (`pid_velocity_fixed_height_controller` and `QuadrotorController`) are used for stability and navigation.
- Computer Vision functionalities are implemented with OpenCV (`cv2`) for object recognition using YOLO model.
- Other libraries include `numpy`, `math`, `threading`, `csv`, `PIL`, `matplotlib`, and custom modules like `ruta_predefinida`, `evita_obstaculos`, `astar3D`, etc.

## Usage

1. Install Webots.
2. Place the .wbt in the Webots worlds directory.
3. Run the Webots simulator with the specified quadrotor model.

## Controls

- UP, DOWN, RIGHT, LEFT: Move in the horizontal plane.
- Q, E: Rotate around yaw.
- W, S: Go up and down.
- A: Enable autonomous mode.
- D: Disable autonomous mode.
- 1, 2, 3: Set predefined targets.
- I: Export data to CSV and generate graphs.

## Autonomous Mode

- The script follows a predefined circular path autonomously.
- Object recognition using YOLO can trigger actions based on detected objects.

## Vision System

- YOLO (You Only Look Once) is used for real-time object detection.
- Camera feed is processed to recognize and react to objects in the environment.

## Additional Features

- Data logging: The script logs positional data and target information into CSV files for further analysis.
- A* algorithm is implemented for path planning.

