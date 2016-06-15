cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Project's name
project(pr2_grasping)

# Colors defined for messages
if(NOT WIN32)
	string(ASCII 27 Esc)
	set(ColorReset	"${Esc}[m")
	set(Red 		"${Esc}[31m")
	set(Green 		"${Esc}[32m")
	set(Yellow 		"${Esc}[33m")
	set(Blue 		"${Esc}[34m")
	set(Magenta 	"${Esc}[35m")
	set(Cyan 		"${Esc}[36m")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	nav_msgs
	actionlib
	pr2_controllers_msgs
	interactive_markers
	moveit_core
	moveit_fake_controller_manager
	moveit_ros_perception
	moveit_ros_planning_interface
)

## Find other dependencies
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)
find_package(yaml-cpp 0.5.2 REQUIRED)
find_package(Boost 1.46 COMPONENTS unit_test_framework REQUIRED)
# find_package(OpenMP REQUIRED)

# Print information related to libraries
message(STATUS "${Yellow}PCL found version ${PCL_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tPCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}${ColorReset}")

message(STATUS "${Yellow}Found OpenCV version ${OpenCV_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tOpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}${ColorReset}")

message(STATUS "${Yellow}Found yaml-cpp version ${yaml-cpp_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tYAML_CPP_INCLUDE_DIR = ${YAML_CPP_INCLUDE_DIR}${ColorReset}")

message(STATUS "${Yellow}Found Boost version ${Boost_LIB_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tBoost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}${ColorReset}")

# Add relevant include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

# Add relevant linking directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Set the build type for PCL
set(PCL_BUILD_TYPE Release)

# Set OpenMP flags to be used
# set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Add extra required definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions("-Wall")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
		roscpp
		std_msgs
		nav_msgs
		actionlib
		pr2_controllers_msgs
		interactive_markers
		moveit_core
		moveit_fake_controller_manager
		moveit_ros_perception
		moveit_ros_planning_interface
)

###########
## Build ##
###########

# Add the directory of the descriptor's libraries
add_subdirectory(src/descriptor_lib)

# Add the include directories of the descriptor's libraries
file(GLOB lib_directories src/descriptor_lib/*)
list(REMOVE_ITEM lib_directories
	${CMAKE_CURRENT_SOURCE_DIR}/src/descriptor_lib/.git
	${CMAKE_CURRENT_SOURCE_DIR}/src/descriptor_lib/test)
foreach(subdir ${lib_directories})
	if (IS_DIRECTORY ${subdir})
		include_directories(${subdir}/include)
	endif()
endforeach()

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executables
add_executable(grasping_node src/grasping_node.cpp)
add_executable(gazebo_pr2_setup src/gazebo_pr2_setup.cpp)

# Set the linking dependencies for the complex applications
set(APPS_LINKING_DEPENDENCIES
	clustering
	descriptor
	factories
	io
	metrics
	utils
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${OpenCV_LIBS}
	${catkin_LIBRARIES})

# Target linking dependencies
target_link_libraries(grasping_node ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(gazebo_pr2_setup ${catkin_LIBRARIES})
