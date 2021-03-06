cmake_minimum_required(VERSION 2.8.3)
project(emc_simulator)
set(main_dir ${PROJECT_SOURCE_DIR}/../../)

find_package(catkin REQUIRED COMPONENTS
    geolib2
    code_profiler
    roslib
    roscpp
    geometry_msgs
    sensor_msgs
)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
    ${main_dir}/include
)

add_library(polypartition
    3rdparty/polypartition/polypartition.cpp
)

add_executable(pico_simulator2
    src/main.cpp
    src/world.cpp
    src/heightmap.cpp
    src/visualization.cpp
    src/lrf.cpp
)
target_link_libraries(pico_simulator2 polypartition ${catkin_LIBRARIES} robolib emc-framework)
