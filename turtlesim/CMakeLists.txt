cmake_minimum_required(VERSION 2.8.3)
project(swarm_sim)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES
                  Color.msg Pose.msg)
add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  SetPen.srv
                  Spawn.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv
                  Betray.srv
                  SwarmList.srv
                  )
generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

set(swarm_sim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)
set(swarm_sim_node_HDRS
  include/turtlesim/turtle_frame.h
)

qt5_wrap_cpp(swarm_sim_node_MOCS ${swarm_sim_node_HDRS})


add_executable(swarm_sim_node ${swarm_sim_node_SRCS} ${swarm_sim_node_MOCS})
target_link_libraries(swarm_sim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(swarm_sim_node swarm_sim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key swarm_sim_gencpp)
add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square swarm_sim_gencpp)
add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic swarm_sim_gencpp)

install(TARGETS swarm_sim_node turtle_teleop_key draw_square mimic
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
