cmake_minimum_required(VERSION 3.8)
project(mirena_sim)
#Compiler options
add_compile_options("-O3")



# find dependencies
find_package(ament_cmake REQUIRED)


# Find ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mirena_common REQUIRED) # For car controls message
find_package(Eigen3 REQUIRED) # For coordinate conversions
find_package(OpenMP REQUIRED) # For lidar optimizations

# Include godot-cpp module
set(GODOTCPP_DISABLE_EXCEPTIONS OFF)
add_subdirectory(godot-cpp)

# Add paths for ROS 2 headers
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${tf2_INCLUDE_DIRS})
include_directories(${tf2_ros_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Source files for the project (your GDExtension and ROS 2 code)
include_directories(src)
set(SOURCES
    src/ros_time.cpp
    src/ros_node3d.cpp
    src/mirena_car.cpp
    src/mirena_cam.cpp
    src/mirena_lidar.cpp
    src/mirena_imu.cpp
    src/mirena_gps.cpp
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
    mirena_common
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

#####################################################[  INSTALL ZONE ]#####################################################################
# Install the simulator for usage with ros2

# Detect architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    # Install the godot engine as executable to lib
    install(PROGRAMS bin/Godot_v4.4-stable_linux.x86_64
        DESTINATION lib/${PROJECT_NAME}
        RENAME godot
    )
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    # Install the godot engine as executable to lib
    install(PROGRAMS bin/Godot_v4.4-stable_linux.arm64
        DESTINATION lib/${PROJECT_NAME}
        RENAME godot
    )
endif()




# Install project files to share directory
install(DIRECTORY
    MirenaSim
    DESTINATION share/${PROJECT_NAME}/
)

# Install the launch script as executable
install(PROGRAMS
    scripts/launch_sim.sh
    DESTINATION lib/${PROJECT_NAME}/
    RENAME MirenaSim
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
