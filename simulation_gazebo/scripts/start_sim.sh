#!/bin/bash
SESSION=sim_hackathon

# Split Panes
tmux -2 new-session -d -s $SESSION
#tmux set pane-border-status top

tmux split-window -h
tmux select-pane -t 0
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v 
tmux select-pane -t 0
tmux split-window -v 
tmux select-pane -t 0
tmux split-window -v 
tmux select-pane -t 4
tmux split-window -v 
tmux select-pane -t 4
tmux split-window -v 
tmux select-pane -t 4
tmux split-window -v 
tmux select-pane -t 4
tmux split-window -v 
tmux select-pane -t 8
tmux split-window -v 
tmux select-pane -t 8
tmux split-window -v 
tmux select-pane -t 8
tmux split-window -v 
tmux select-pane -t 8
tmux split-window -v 

# Exec Commands
#tmux send-keys -t 6 './pub_att.sh' C-m
tmux send-keys -t 0 "roslaunch bringup_gazebo competition.launch use_sitl:=true world_name:=$(rospack find subt_custom_gazebo)/worlds/local_planner_test.world" C-m
#tmux send-keys -t 0 "roslaunch bringup_gazebo competition.launch use_sitl:=true world_name:=$(rospack find subt_custom_gazebo)/worlds/edgar_mine.world" C-m
sleep 5
tmux send-keys -t 2 "roslaunch bringup_gazebo spawn_rollo.launch" C-m
sleep 2
tmux send-keys -t 1 'roslaunch bringup_rollo mobility.launch simulation:=true use_odometry_ground_truth:=true' C-m
tmux send-keys -t 3 "rostopic echo /rollo/mavros/state" C-m
tmux send-keys -t 4 'rosservice call /rollo/mavros/cmd/arming "value: true"'
tmux send-keys -t 5 'sleep 6; rosrun mavros mavsys -n rollo/mavros mode -c "OFFBOARD"' C-m
tmux send-keys -t 6 'roscd simulation_px4_sitl/scripts; ./run_mobility_cli.sh' C-m
tmux send-keys -t 7 'sleep 6;source /opt/ros/melodic/setup.bash; node_manager' C-m
tmux send-keys -t 8 'roscd local_planner_hybrid/rviz; rviz -d local_planner.rviz' C-m
tmux send-keys -t 9 '' C-m
tmux send-keys -t 10 '' C-m
tmux -2 attach-session -t $SESSION
