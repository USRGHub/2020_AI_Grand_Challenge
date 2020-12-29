# UGV Project

Make the UGV model using URDF

# Create Package

We need three main packages, ugv_control, ugv_description, ugv_gazebo

    cd ~/catkin_ws/src/
    catkin_create_pkg ugv_control
    catkin_create_pkg ugv_description
    catkin_create_pkg ugv_gazebo

because it is not available as a deb.

# Creating UGV World

First of all, create the some subfolders in the gazebo package

    roscd ugv_gazebo
    mkdir launch worlds

Then, create the a new world file in the worlds folder

    cd worlds
    gedit ugv.world
    
write the basic code as follow

    <?xml version="1.0"?>
    <sdf version="1.5">
    <world name="ugv">
    </world>
    </sdf>
    
Here you could directly add models and object with their position. Also the laws of physics may be defined in a world. This is an important step to understand, because in this file you could also attach a specific plugin to an object. The plugin itself contains ROS and Gazebo specific code for more complex behaviors.

At first we just want to add some basic objects, like a ground and a basic illumination source inside the world tag.

    <include>
        <uri>model://sun</uri>
    </include>

    <include>
        <uri>model://ground_plane</uri>
    </include>
    
Change to the launch directory of your project:



end.
