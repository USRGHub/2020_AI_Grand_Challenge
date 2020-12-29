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
    
    roscd ugv_gazebo/launch
    
Create a new file:

    gedit ugv_world.launch

and insert:

    <launch>      
        <include file="$(find gazebo_ros)/launch/empty_world.launch">        
            <arg name="world_name" value="$(find ugv_gazebo)/worlds/ugv.world"/>        
            <arg name="gui" value="true"/>      
        </include>    
    </launch>
    
This launch file will just execute a default launch file provided by Gazebo, and tell it to load our world file and show the Gazebo client. You can launch it by doing:

    roslaunch ugv_gazebo ugv_world.launch

Now you should see the gazebo server and the gui starting with a world that contains a ground plane and a sun (which is not obviously visible without objects). If not, it can be that there are some connections problems with the server.

# Create the Robot Model (URDF)

In ~/catkin_ws/src/ugv_description/urdf, there will be four files:

ugv.xacro: primary file that loads the other three files and contains only URDF items like joints and links

ugv.gazebo: contains gazebo-specific labels that are wrapped within gaz

materials.xacro: maps strings to colors

macros.xacro: macros to help simplify

The more accurate you want to model your robot the more time you need to spend on the design. In the next image you see a developing process of a robot model, from a simple cube to a differential drive robot. You can also check the ROS tutorial about the robot.

The Universal Robotic Description Format (URDF) is an XML file format used in ROS as the native format to describe all elements of a robot. Xacro (XML Macros) is an XML macro language. It is very useful to make shorter and clearer robot descriptions.

Ok so first we need to go into our description package and create the urdf subfolder and the description file:

    roscd ugv_description
    mkdir urdf
    cd urdf
    gedit ugv.xacro

Let’s define some physical properties for our robot, mainly the dimensions of the chassis, the caster wheel, the wheels:

    <xacro:property name="PI" value="3.1415926535897931"/>

    <xacro:property name="chassisHeight" value="0.1"/>
    <xacro:property name="chassisLength" value="0.4"/>
    <xacro:property name="chassisWidth" value="0.2"/>
    <xacro:property name="chassisMass" value="50"/>

    <xacro:property name="casterRadius" value="0.05"/>
    <xacro:property name="casterMass" value="5"/>

    <xacro:property name="wheelWidth" value="0.05"/>
    <xacro:property name="wheelRadius" value="0.1"/>
    <xacro:property name="wheelPos" value="0.2"/>
    <xacro:property name="wheelMass" value="5"/>

We will also include three files :

    <xacro:include filename="$(find ugv_description)/urdf/ugv.gazebo" />
    <xacro:include filename="$(find ugv_description)/urdf/materials.xacro" />
    <xacro:include filename="$(find ugv_description)/urdf/macros.xacro" />

Every file has to contain the robot tag and everything we put in them should be in this tag.

Now we want to add a rectangular base for our robot. Insert this within the robot tag of ugv.xacro:

    <link name='chassis'>
      <collision> 
        <origin xyz="0 0 ${wheelRadius}" rpy="0 0 0"/> 
        <geometry> 
          <box size="${chassisLength} ${chassisWidth} ${chassisHeight}"/> 
        </geometry> 
      </collision>
      <visual> 
        <origin xyz="0 0 ${wheelRadius}" rpy="0 0 0"/> 
        <geometry> 
          <box size="${chassisLength} ${chassisWidth} ${chassisHeight}"/> 
        </geometry> 
        <material name="orange"/>
      </visual>
      <inertial> 
        <origin xyz="0 0 ${wheelRadius}" rpy="0 0 0"/> 
        <mass value="${chassisMass}"/> 
        <box_inertia m="${chassisMass}" x="${chassisLength}" y="${chassisWidth}" z="${chassisHeight}"/>
      </inertial>
    </link>

As you can see, we have three tags for this one box, where one is used to the collision detection engine, one to the visual rendering engine and the last to the physic engine. Most of the time they are the same, except when you have complicated and beautiful visual meshes.

And this in the “materials.xacro” :

    <material name="black">
      <color rgba="0.0 0.0 0.0 1.0"/>
    </material>

    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>

    <material name="green">
      <color rgba="0.0 0.8 0.0 1.0"/>
    </material>

    <material name="grey">
      <color rgba="0.2 0.2 0.2 1.0"/>
    </material>

    <material name="orange">
      <color rgba="${255/255} ${108/255} ${10/255} 1.0"/>
    </material>

    <material name="brown">
      <color rgba="${222/255} ${207/255} ${195/255} 1.0"/>
    </material>

    <material name="red">
      <color rgba="0.8 0.0 0.0 1.0"/>
    </material>

    <material name="white">
      <color rgba="1.0 1.0 1.0 1.0"/>
    </material> 

As you can see, we add more than just the color we wanted, this is for convenience. Now, we can leave this file alone and use any color we want.

Add this in the macros.xacro file, within the robot tag :

    <macro name="cylinder_inertia" params="m r h">
      <inertia  ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
        iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
        izz="${m*r*r/2}"
      />
    </macro>

    <macro name="box_inertia" params="m x y z">
      <inertia  ixx="${m*(y*y+z*z)/12}" ixy = "0" ixz = "0"
        iyy="${m*(x*x+z*z)/12}" iyz = "0"
        izz="${m*(x*x+z*z)/12}"
      />
    </macro>

    <macro name="sphere_inertia" params="m r">
      <inertia  ixx="${2*m*r*r/5}" ixy = "0" ixz = "0"
        iyy="${2*m*r*r/5}" iyz = "0"
        izz="${2*m*r*r/5}"
      />
    </macro>

The physic engine does not accept a base_link with inertia. It is then useful to add a simple link without inertia and make a joint between it and the chassis. Add this before the chassis link in the ugv.xacro file :

    <link name="footprint" />

    <joint name="base_joint" type="fixed">
      <parent link="footprint"/>
      <child link="chassis"/>
    </joint>

In order to start gazebo with our model, we have to modify the previously created launch file ugv_world.launch by adding the following two tags in the launch tag:


    <!-- urdf xml robot description loaded on the Parameter Server, converting the xacro into a proper urdf file-->
    <param name="robot_description" command="$(find xacro)/xacro.py '$(find ugv_description)/urdf/ugv.xacro'" />

    <!-- push robot_description to factory and spawn robot in gazebo -->
    <node name="ugv_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
     args="-urdf -param robot_description -model ugv" />


The first tag will first call the xacro script to convert of xacro description into an actual URDF. This URDF is then inserted into a ROS parameter called “robot_description” (this is a standard name used by many ROS tools).


end.
