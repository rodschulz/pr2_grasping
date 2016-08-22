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
	roscpp
	rospy
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

# Add extra required definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions("-Wall")

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
		roscpp
		rospy
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

include_directories(src/grasping_lib/)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executables
add_executable(gazebo_pr2_labeler src/gazebo_pr2_labeler.cpp)
add_executable(gazebo_pr2_setup src/gazebo_pr2_setup.cpp)
add_executable(pr2_grasper src/pr2_grasper.cpp)

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
target_link_libraries(gazebo_pr2_setup ${catkin_LIBRARIES})
target_link_libraries(gazebo_pr2_labeler ${APPS_LINKING_DEPENDENCIES})
target_link_libraries(pr2_grasper ${APPS_LINKING_DEPENDENCIES})
