file(GLOB GRASPING_SRC
	"src/*.cpp"
)

include_directories(include)

add_library(grasping ${GRASPING_SRC})
target_link_libraries(grasping 
		utils
		factories
		${PCL_LIBRARIES}
		${OpenCV_LIBS}
		${catkin_LIBRARIES})
