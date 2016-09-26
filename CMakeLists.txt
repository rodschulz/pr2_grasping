cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0045 OLD)
cmake_policy(SET CMP0054 OLD)

# Project's name
project(pr2_grasping)

# Enable or disable the use of OpenMP
set(useOpenMP FALSE)

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
	message_generation
	roscpp
	rospy
	actionlib
	std_msgs
	geometry_msgs
	nav_msgs
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

# Print information related to libraries
message(STATUS "${Yellow}PCL found version ${PCL_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tPCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS}${ColorReset}")

message(STATUS "${Yellow}Found OpenCV version ${OpenCV_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tOpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}${ColorReset}")

message(STATUS "${Yellow}Found yaml-cpp version ${yaml-cpp_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tYAML_CPP_INCLUDE_DIR = ${YAML_CPP_INCLUDE_DIR}${ColorReset}")

message(STATUS "${Yellow}Found Boost version ${Boost_LIB_VERSION}${ColorReset}")
message(STATUS "${Cyan}\tBoost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}${ColorReset}")

# Add custom message files
add_message_files(
	FILES
		GraspingPoint.msg
		GraspingPointArray.msg
)

# Generate the custom messages
generate_messages(
	DEPENDENCIES
		std_msgs
		geometry_msgs
)

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

# Add extra required definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions("-Wall")

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
		message_runtime
		roscpp
		rospy
		actionlib
		std_msgs
		nav_msgs
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

include_directories(src/grasping_lib/)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executables
### App nodes ###
add_executable(gazebo_pr2_labeler src/gazebo_pr2_labeler.cpp)
add_executable(gazebo_pr2_setup src/gazebo_pr2_setup.cpp)
add_executable(pr2_grasper src/pr2_grasper.cpp)
### Example nodes ###
add_executable(example_gp src/example_gp.cpp)
add_executable(example_ac src/example_ac.cpp)
### Test nodes ###
add_executable(test_gp src/test_gp.cpp)
add_executable(test_ac src/test_ac.cpp)
add_executable(pick_place_tutorial src/pick_place_tutorial.cpp)

# Set the linking dependencies for the complex applications
set(APPS_LINKING_DEPENDENCIES
	clustering
	descriptor
	factories
	io
	utils
	${PCL_LIBRARIES}
	${YAML_CPP_LIBRARIES}
	${OpenCV_LIBS}
	${catkin_LIBRARIES})

# Target linking dependencies
### App nodes ###
target_link_libraries(gazebo_pr2_setup ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(gazebo_pr2_labeler ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(pr2_grasper ${APPS_LINKING_DEPENDENCIES})
### Example nodes ###
target_link_libraries(example_gp ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(example_ac ${APPS_LINKING_DEPENDENCIES})
### Test nodes ###
target_link_libraries(test_gp ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(test_ac ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(pick_place_tutorial ${APPS_LINKING_DEPENDENCIES})
