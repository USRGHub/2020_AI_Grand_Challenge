# UGV Project

Make the UGV model using URDF

# Dependencies

Currently in jade and kinetic it is necessary to

    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git

because it is not available as a deb.

If using kinetic, get the kinetic branch:

     cd gazebo_ros_pkgs
     git checkout remotes/origin/kinetic-devel

# gazebo simulation

    roslaunch carbot_gazebo_control carbot_gazebo_control.launch

If it is the first time gazebo has launched, this may take a long time before you see

    [Spawn status: SpawnModel: Successfully spawned model]

and the model appears in rviz and gazebo.

Gazebo frequently doesn't exit cleanly after pressing ctrl-c, you may have to kill the roscore and restart it to run again.

If it does exit cleanly it is a good idea to:
