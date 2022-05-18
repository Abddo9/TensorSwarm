source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:./devel/lib

roscore &

catkin_make && argos3 -c src/TensorSwarm/argos_worlds/4_way.argos &
argos3 -c src/TensorSwarm/argos_worlds/4_way.argos &
argos3 -c src/TensorSwarm/argos_worlds/4_way.argos &
argos3 -c src/TensorSwarm/argos_worlds/4_way.argos &

python3 src/TensorSwarm/scripts/new/RunExperiment4Way.py
