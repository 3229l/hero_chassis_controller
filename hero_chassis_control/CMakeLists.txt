cmake_minimum_required(VERSION 3.10)
project(hero_chassis_control)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin  REQUIRED
    COMPONENTS
    roscpp
    rospy
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    realtime_tools
    geometry_msgs
    control_msgs
    control_toolbox
    dynamic_reconfigure
    nav_msgs
    tf
    teleop_twist_keyboard
    )

generate_dynamic_reconfigure_options(
    config/pid.cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
    include
        ${dynamic_reconfigure_INCLUDE_DIRS}
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    rospy
    roslint
    controller_interface
    hardware_interface
    forward_command_controller
    pluginlib
    realtime_tools
    geometry_msgs
    control_msgs
    control_toolbox
    dynamic_reconfigure
    nav_msgs
    tf
    DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${dynamic_reconfigure_INCLUDE_DIRS}
#    ${PROJECT_SOURCE_DIR}/devel/include
    # Set manually because Eigen sets a non standard INCLUDE DIR
)

## Declare cpp executables
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_executable(teleop_twist_keyboard src/teleop_twist_keyboard.cpp)
target_link_libraries(teleop_twist_keyboard ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(
#    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark cpp header files for installation
#install(
#    DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.hpp"
#)
#
## Mark other files for installation
#install(
#    DIRECTORY doc
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
##############
### Testing ##
##############
#
#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

##########################
## Static code analysis ##
##########################

roslint_cpp()


