cmake_minimum_required(VERSION 3.9)

project(ufoexplorer 
	# VERSION 1.0.0
	# DESCRIPTION "UFOExplorer: TBD"
	# LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	collision_avoidance
	dynamic_reconfigure
	geometry_msgs
	roscpp
	tf2
	tf2_ros
	ufomap_msgs
	ufomap_ros
	visualization_msgs
)

find_package(ufomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.71.0 REQUIRED)

generate_dynamic_reconfigure_options(
  # cfg/PlannerTree.cfg
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS} ${ufomap_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  # LIBRARIES ufoexplorer ${catkin_LIBRARIES} ${ufomap_LIBRARIES}
#  CATKIN_DEPENDS exploration_planner_msgs roscpp geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ufomap_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(ufoexplorer_node
	src/explorer_node.cpp
  src/ufoexplorer/explorer.cpp
	src/ufoexplorer/graph_search.cpp
	src/ufoexplorer/map.cpp
	src/ufoexplorer/tsp_solver.cpp
)
add_dependencies(ufoexplorer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_compile_features(ufoexplorer_node
# 	PUBLIC 
# 		cxx_std_17
# )
target_link_libraries(ufoexplorer_node
  ${catkin_LIBRARIES}
  UFO::Map
)

add_executable(trajectory_node
	src/trajectory_node.cpp
)
add_dependencies(trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_compile_features(ufoexplorer_node
# 	PUBLIC 
# 		cxx_std_17
# )
target_link_libraries(trajectory_node
  ${catkin_LIBRARIES}
  UFO::Map
)

# add_executable(process_data_node
# 	src/process_data_node.cpp
# )
# add_dependencies(process_data_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # target_compile_features(ufoexplorer_node
# # 	PUBLIC 
# # 		cxx_std_17
# # )
# target_link_libraries(process_data_node
#   ${catkin_LIBRARIES}
#   UFO::Map
# )

# add_executable(evaluation_node
# 	src/evaluation_node.cpp
# )
# add_dependencies(evaluation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # target_compile_features(ufoexplorer_node
# # 	PUBLIC 
# # 		cxx_std_17
# # )
# target_link_libraries(evaluation_node
#   ${catkin_LIBRARIES}
#   UFO::Map
# )

# add_executable(evaluation_regex_node
# 	src/evaluation_regex_node.cpp
# )
# add_dependencies(evaluation_regex_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # target_compile_features(ufoexplorer_node
# # 	PUBLIC 
# # 		cxx_std_17
# # )
# target_link_libraries(evaluation_regex_node
#   ${catkin_LIBRARIES}
#   UFO::Map
# )

# add_executable(evaluation_generate_result_node
# 	src/evaluation_generate_result_node.cpp
# )
# add_dependencies(evaluation_generate_result_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # target_compile_features(ufoexplorer_node
# # 	PUBLIC 
# # 		cxx_std_17
# # )
# target_link_libraries(evaluation_generate_result_node
#   ${catkin_LIBRARIES}
#   UFO::Map
# )

# add_executable(evaluation_reconstruction_node
# 	src/evaluation_reconstruction_node.cpp
# )
# add_dependencies(evaluation_reconstruction_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_compile_features(ufoexplorer_node
# 	PUBLIC 
# 		cxx_std_17
# )
# target_link_libraries(evaluation_reconstruction_node
#   ${catkin_LIBRARIES}
#   UFO::Map
# )

install(TARGETS ufoexplorer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )