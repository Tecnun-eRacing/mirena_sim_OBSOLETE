cmake_minimum_required(VERSION 3.8)
project(mirena_sim)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-O3")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# Find ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mirena_msg REQUIRED) # For car controls message

find_package(OpenMP REQUIRED)

# Include godot-cpp module
add_subdirectory(godot-cpp)


# Add paths for ROS 2 headers
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${tf2_INCLUDE_DIRS})
include_directories(${tf2_ros_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})

# Source files for the project (your GDExtension and ROS 2 code)
include_directories(src)
set(SOURCES
    src/ros_time.cpp
    src/ros_node3d.cpp
    src/mirena_car.cpp
    src/mirena_cam.cpp
    src/mirena_lidar.cpp
    src/mirena_imu.cpp
    src/register_types.cpp
)

# Create the shared library for the GDExtension
add_library(mirena_ros SHARED ${SOURCES})

# Ensure ROS 2 libraries are found
ament_target_dependencies(mirena_ros
    rclcpp
    tf2
    tf2_ros
    std_msgs
    sensor_msgs
    geometry_msgs
    mirena_msg
)

# Link Godot GDExtension, Godot CPP bindings, ROS 2 libraries
target_link_libraries(mirena_ros
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    godot::cpp
    OpenMP::OpenMP_CXX
)


# Copy generated library and export the godot project
add_custom_command(
    TARGET mirena_ros POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mirena_ros> ${CMAKE_SOURCE_DIR}/MirenaSim/LibMirena/libmirena_sim.so
    COMMENT "Copying the built shared library to godot LibMirena extension folder"
)


#Install the simulator for usage
#Copy the godot project to share on the install folder
install(DIRECTORY 
MirenaSim
DESTINATION share/${PROJECT_NAME}/
)

#Install the launchfiles
install(DIRECTORY 
launch
DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
