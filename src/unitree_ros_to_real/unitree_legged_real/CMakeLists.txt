cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
    nav_msgs
)

catkin_package()

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    /usr/local/include/eigen3
)

set(CMAKE_CXX_FLAGS "-O3")

# set(SDK_LOCAL_PATH mounted_home/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_legged_sdk_from_inside_robot) # amarco added (amarco laptop - ubuntu VM)
# set(SDK_LOCAL_PATH vicon_ws/src/unitree_legged_sdk_from_inside_robot) # eli added (Eli-Ubuntu-Laptop)
# set(SDK_LOCAL_PATH catkin_real_robot_ws/src/unitree_legged_sdk_from_inside_robot) # amarco added (amarco laptop - ubuntu, robot lab)

# include_directories(/home/$ENV{USER}/${SDK_LOCAL_PATH}/include) # amarco edited
# link_directories(/home/$ENV{USER}/${SDK_LOCAL_PATH}/lib) # amarco edited
include_directories(../../unitree_legged_sdk_from_inside_robot/include) # amarco edited
link_directories(../../unitree_legged_sdk_from_inside_robot/lib) # amarco edited
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

# add_executable(lcm_server /home/$ENV{USER}/${SDK_LOCAL_PATH}/examples/lcm_server.cpp) # amarco edited
# target_link_libraries(lcm_server ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(lcm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# # ROS nodes: Not using their LCM threads
# add_executable(position_holder nodes/position_holder.cpp src/position_holder_control_loop.cpp /home/$ENV{USER}/${SDK_LOCAL_PATH}/python_interface_go1/src/real_robot_interface_go1.cpp)
# target_link_libraries(position_holder ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(position_holder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(stand_up nodes/stand_up.cpp)
# target_link_libraries(stand_up ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(stand_up ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# # Based on nodes/examples_unitree/position_mode.cpp; using their LCM threads
# add_executable(position_holder_lcm nodes/position_holder_lcm.cpp)
# target_link_libraries(position_holder_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(position_holder_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(stand_up_lcm nodes/stand_up_lcm.cpp)
# target_link_libraries(stand_up_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(stand_up_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_real_robot_interface_go1_highlevel /home/$ENV{USER}/${SDK_LOCAL_PATH}/python_interface_go1/src/real_robot_interface_go1_highlevel.cpp nodes/ros_real_robot_interface_go1_highlevel.cpp)
add_executable(ros_real_robot_interface_go1_highlevel ../../unitree_legged_sdk_from_inside_robot/python_interface_go1/src/real_robot_interface_go1_highlevel.cpp nodes/ros_real_robot_interface_go1_highlevel.cpp)
target_link_libraries(ros_real_robot_interface_go1_highlevel ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_real_robot_interface_go1_highlevel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# # ROS nodes - Unitree examples

# add_executable(position_lcm nodes/examples_unitree/position_mode.cpp)
# target_link_libraries(position_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(position_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(velocity_lcm nodes/examples_unitree/velocity_mode.cpp)
# target_link_libraries(velocity_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(velocity_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(torque_lcm nodes/examples_unitree/torque_mode.cpp)
# target_link_libraries(torque_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(torque_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(walk_lcm nodes/examples_unitree/walk_mode.cpp)
# target_link_libraries(walk_lcm ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(walk_lcm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(walk_lcm_mod nodes/examples_unitree/walk_mode_mod.cpp)
# target_link_libraries(walk_lcm_mod ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(walk_lcm_mod ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Python nodes:

catkin_install_python(PROGRAMS nodes/python/node_data_collection.py nodes/python/node_robot_state_creation.py nodes/python/node_walk_open_loop.py nodes/python/node_go_to_lowlevel.py nodes/python/node_ood_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
