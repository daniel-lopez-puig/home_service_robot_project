# Where_am_I
Where_am_I is basic robot simulation focused on localization. This project can be used as a base/template project to start your own modified project of localization using amcl pkg.

It is basically composed by three different components:
- The simulation of the robot that holds a world and a very simple robot inside.
- Configuration for amcl package that enables the robot to get information from laser and odometry and outputs the localization.
- A map of the world well configured

# Installation
This simulation have been created and tested in:
- [Ubuntu 16.04](https://ubuntu.com/download/desktop) (supports Ubuntu 16.04) 
- [ROS Kinetic](http://wiki.ros.org/melodic/Installation/Ubuntu) (supports ROS kinetic, with melodic has some issues)
- [Gazebo 7.0](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) (supports Gazebo 7.0 or superior)

Install some pkgs

```bash
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-amcl
```

### Create a catkin workspace to compile and run the simulation

```bash
mkdir -p catkin_ws/src # create 2 folders
cd catkin_ws/src
catkin_init_workspace # createa CMakeLists.txt
cd catkin_ws # go to main folder
catkin_make # create some automatic folders and files
cd src # go to source folder
git clone https://github.com/daniel-lopez-puig/where_am_i_project.git #clone this repository
mv where_am_i_project/* . && rm -r where_am_i
cd .. # go back to catkin_ws
catkin_make
```

### Run the simulation and ROS packages
This will open two windows, gazebo and rviz.

```bash
cd catkin_ws
source devel/setup.bash
roslaunch where_am_i amcl.launch
```

In gazebo you can see all the simulated world with the robot and a building.
![gazebo_world](readme_images/gazebo_with_ball_chase_it.png)

On the other hand you will have a rviz window thats shows what the robot sees using the laser (colored dots) and the front camera (bottom right image).
![gazebo_world](readme_images/rviz_chase_it.png)

### Send the robot to a desired position
Move the robot sending a goal inside the map

![gazebo_gif](readme_images/chasing_ball.gif)

Or alternatively open a new terminal and move the robot manually:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
# Structure
This package is mainly composed by two folders, **teleop_twist_keyboard** (responsible to drive the robot manually) and the **where_am_i** that simulates the robot, launch all localization files proper configured.

![Tree](readme_images/chase_it_structure.png)

# Contribute

This project have been done entirely for me while coursing the  [Roftware Software Engineer nando degree program](https://www.udacity.com/course/robotics-software-engineer--nd209) in Udacity. Please feel free to fork and create your own branch with your personalized projects.

# License

Feel free to use this repository to create your own simulation following the [MIT license attached](LICENSE).

# Contact

Do not hesitate to contact me via mail (daniel.lopez.puig@gmail.com) or by [Linkedin](https://www.linkedin.com/in/daniel-lopez-puig/) to give any suggestion or possible colaboration project realated to robotics.
