# mirena_sim
MirenaSim is a godot simulation enviroment for formula student driverless system development, it implements the various required sensors to start off with ros2 tinkering.

## Current Features
- Fully fledged Car Model, implementing, basic tire friction, suspension dynamics.
- mirena_car -> Car controls node
- mirena_lidar -> An implementation of a lidar using multithreaded raycasting
- mirena_imu -> An implementation of an IMU providing standard ros2 twist pose and accel messages
- mirena_cam -> A viewport/camera broadcaster
