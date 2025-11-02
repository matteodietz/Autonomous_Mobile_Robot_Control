# Autonomous_Mobile_Robot_Control

## Project 1: Designing Nav2 Controller and Planner Plugins

Goal of this project was to let a turtlebot navigate along multiple waypoints in an environment with obstacles. To achieve this, we designed Nav2 controller and planner plugins. We aimed to improve a generic pure pursuit controller using various heuristics. Using this controller, a robot should follow a path found by an A* planner and smoothened by a Savitsky-Golay algorithm in both a virtual and a physical environment.

More information about this project can be found in the [complete project report (PDF)](./p1_maze_solver_robot.pdf).

## Project 2: Drone Simulation With Simplified Kalman Filter

Goal of this project was to let a quadcopter navigate along multiple waypoints in a virtual environment. To achieve this, we designed controller and estimator plugins. For the controller, we implemented a holonomic pure pursuit controller and for the estimation, we implemented a Kalman filter that uses sonar, GPS, magnetic compass and barometer measurements to predict and correct the quadcopters pose. We the aimed to improve the pure pursuit controller from project 1 using simple heuristics. Using this controller and an improved Kalman filter algorithm, the drone should take off, follow a moving turtlebot and fly to the turtlebots goal points in an alternating fashion and then land again.

More information about this project can be found in the [complete project report (PDF)](./p2_ekf_quadcopter.pdf).
