cmake_minimum_required(VERSION 3.8)
project(mirena_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# Find ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mirena_msg REQUIRED) # For car controls message
find_package(OpenMP REQUIRED)

# Include godot-cpp module
add_subdirectory(godot-cpp)


# Add paths for ROS 2 headers
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})

# Source files for the project (your GDExtension and ROS 2 code)
include_directories(src)
set(SOURCES
    src/mirena_car.cpp
    src/mirena_cam.cpp
    src/mirena_lidar.cpp
    src/register_types.cpp
)

# Create the shared library for the GDExtension
add_library(mirena_ros SHARED ${SOURCES})

# Ensure ROS 2 libraries are found
ament_target_dependencies(mirena_ros
    rclcpp
    std_msgs
    sensor_msgs
    mirena_msg
)

# Link Godot GDExtension, Godot CPP bindings, ROS 2 libraries
target_link_libraries(mirena_ros
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    godot::cpp
    OpenMP::OpenMP_CXX
)


# Copy generated library and export the godot project
add_custom_command(
    TARGET mirena_ros POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mirena_ros> ${CMAKE_SOURCE_DIR}/MirenaSim/LibMirena/libmirena_sim.so
    COMMENT "Copying the built shared library to godot LibMirena extension folder"
)

# Set the godot export preset
set(EXPORT_PRESET "Linux")

# Custom target to export the Godot project
add_custom_target(${PROJECT_NAME} ALL
    COMMAND  mkdir -p "${CMAKE_BINARY_DIR}/MirenaSim" && godot --export-debug "${EXPORT_PRESET}" "${CMAKE_BINARY_DIR}/MirenaSim/MirenaSim"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/MirenaSim
    COMMENT "Exporting Godot project with built shared library to ${CMAKE_BINARY_DIR}"
)

add_dependencies(${PROJECT_NAME} mirena_ros)

#Install simulator for usage

#Install the simulator for usage
install(DIRECTORY 
${CMAKE_BINARY_DIR}/MirenaSim
DESTINATION share/${PROJECT_NAME}/
FILE_PERMISSIONS 
        OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
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
