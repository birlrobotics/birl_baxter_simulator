cmake_minimum_required(VERSION 2.8.3)
project(birl_sim_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  gazebo_ros
  gazebo_msgs
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  )

# Generate services in the 'srv' folder
add_service_files(
  FILES Add_Gazebo_Model.srv
  Go_To_Start_Position.srv
  Go_To_Position.srv)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  gazebo_ros
  gazebo_msgs
  baxter_sim_kinematics
  )

#############
## Install ##
#############

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch )
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

