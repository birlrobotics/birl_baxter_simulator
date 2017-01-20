# birl_baxter_simulator

# Purpose
This package is used for applying Baxter from rethink robotics inc in experiments of our lab, [BIRL](https://github.com/birlrobotics/birl_baxter/wiki).
It is an the extension from [baxter_simulator](https://github.com/RethinkRobotics/baxter_simulator), a package from the baxter sdk.

# New Features

![baxter](https://github.com/birlrobotics/birl_baxter_common/blob/master/media/full.png)

The environment for BIRL experiments includes as follows:
  1.  Asus xtion camera sensor and camera frame
  2.  Table
  3.  WACOH force torque sensor and its design
  4.  A in house designed plastic camera gripper holder used for a camera snap assembly task
  5.  A few other object models for the tasks

A number of tasks that we currently offer:
 1. A pick and place box.
 2. A one-arm box assembly
 3. A one-arm snap assembly using a plastic camera mold with male and female parts. 

For more details, you can click: [New Feature documentation](https://github.com/birlrobotics/birl_baxter_common/wiki/New-Features-documentation)

# Dependencies
This package depends on baxter_common and baxter_simulator. So you must install the [baxter_sdk](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) and [baxter_simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation).

# Installation
1. Install **baxter_sdk** and **baxter_simulator**

   - [baxter_sdk](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) 

   - [baxter_simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation).

  Please follow the installation of **baxter_sdk** and **baxter_simulator** on the Rethink Robotics official website.  

  You can check it after installation to find if you installed properly by a simple command: 
#
    roslaunch baxter_gazebo baxter_world.launch 

 
2. Install the **birl_baxter_common** and **birl_baxter_simulator** repositories:
````
$ cd {your_baxter_work_space}/src // in my case ~/ros/indigo/baxter_ws/src
git clone https://github.com/birlrobotics/birl_baxter_common.git
git clone https://github.com/birlrobotics/birl_baxter_simulator.git
cd ..
catkin_make
````
 And finished! Enjoy it.

# Launch Examples
=======================================================================================
**Task1**: A one-arm box assembly
    
**[Description]**: Try to assembly the male box on the female box with one arm.
````
roslaunch birl_sim_examples place_box.launch
````
=======================================================================================
**Task2:** A one-arm snap assembly using a plastic camera mold with male and female parts

**[Description]:** Try to assembly the male snap on the female snap using a plastic camera mold with one arm
````
roslaunch birl_sim_examples place_snap.launch
````
=======================================================================================
**Task3:** Pick and place the box

**[Description]:** Pick and place the box on the table with two goals.
````
roslaunch birl_sim_examples pick_n_place_box.launch
````
=======================================================================================
**Task4:** pick and place smach servie scripts
**[Description]**: using state machine to achieve pick and place box task
````
roslaunch birl_sim_examples pick_n_place_box_smach_service.launch
````

But we recommend you to open nodes one by one.

Maybe you will find issue when launching the smach_viewer, please read the [ros_issue](http://answers.ros.org/question/172688/ros-indigo-cannot-show-graph-view-on-smach_viewer/) and do some simple change work.

For more information, you can check the documentation [smach_service](https://github.com/birlrobotics/birl_baxter_simulator/wiki/smach-service)
  
=======================================================================================
**Task5:** pick and place servie scripts
**[Description]**: using service to achieve pick and place box task
````
roslaunch birl_sim_examples pick_n_place_box_service.launch
````

# Learn More
If you want to look into the raw files, you can find the files blow:

Environment raw **launch** files, you can find the launches files here:
````
roscd birl_baxter_description/launch
````

Manipulation **examples** file, you can find here:
````
roscd birl_sim_examples/scripts
````

**URDF** file, you can find here
````
roscd birl_baxter_description/urdf
````

# Mechanical Design Source
To access the repo with the hardware design of the FT sensor, grippers, and objects, see: [birl_baxter_hands](https://github.com/birlrobotics/birl_baxter_hands)

# Learn More About BIRL
Our central baxter repository has more information about a variety of packages that we support: [BIRL Baxter](https://github.com/birlrobotics/birl_baxter/wiki)
