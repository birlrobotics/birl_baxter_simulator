# birl_baxter_simulator

# Purpose
This package is used for applying Baxter robot from rethink robotics inc to experiments of our lab,BIRL.
The package is of the extension from baxter_common,a package from baxter sdk.

# New Feature

![baxter](https://github.com/birlrobotics/birl_baxter_common/blob/master/media/baxter.png)

The environment for BIRL includes as followed:
  1.  Asus xtion camera sensor and camera frame
  2. Table
  3. Force torque sensor and its design
  4. camera gripper and holder designed for camera snap assembly task
  5. some object model for task

For more detail, you can click [New Feature documentation](https://github.com/birlrobotics/birl_baxter_common/wiki/New-Features-documentation)

# Dependence
This package depend on baxter_common,baxter_simulator.So you must install [baxter_sdk](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) and [baxter_simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation).make sure you instatll baxter_sdk and baxter_simulator properly.

# Installation
1.First of all you should install 
- [baxter_sdk](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) 
- [baxter_simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation).

  Please follow the installation wiki on rethink robotics official website  

  you can try this to check if your baxter gazebo environment is setup well
````
roslaunch baxter_gazebo baxter_launch 
```` 
 
2.Install the repo birl_baxter_common and birl_baxter_simulator
````
cd {your_baxter_work_space}/src 
  (got to your baxter_ws,in my case, cd ~/ros/indigo/baxter_ws/src)
git clone https://github.com/birlrobotics/birl_baxter_common.git
git clone https://github.com/birlrobotics/birl_baxter_simulator.git
cd ..
catkin_make
````
finish  installing, enjoy!

# Launch examples
Task1: place the male box to female box
````
roslaunch birl_sim_examples place_box.launch
````

Task2: place the male snap to female snap
````
roslaunch birl_sim_examples place_snap.launch
````

Task3: pick and place the female box
````
roslaunch birl_sim_examples pick_n_place_box.launch
````



If you want to look into the raw files, you can find the files blow:

for the environment raw launch files, you can find the launches files here:
````
roscd birl_baxter_description/launch
````

for the manipulation examples file, you can find here
````
roscd birl_sim_examples/scripts
````

for the urdf file, you can find here
````
roscd birl_baxter_description/urdf
````


# Mechanical design source
Here are the repo which contain our mechanical design source[birl_baxter_hands](https://github.com/birlrobotics/birl_baxter_hands)

# Learn more about BIRL
Welcome to [BIRL](https://github.com/birlrobotics/birl_baxter/wiki)
