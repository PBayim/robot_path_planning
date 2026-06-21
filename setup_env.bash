# Source this before working in this workspace:
#   source setup_env.bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
