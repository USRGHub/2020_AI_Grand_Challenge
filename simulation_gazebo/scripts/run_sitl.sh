#!/bin/bash

PROJECT_DIR="$(rospack find simulation_px4_scout)"

cd $PROJECT_DIR/Firmware && \
  no_sim=1 make px4_sitl_default gazebo
