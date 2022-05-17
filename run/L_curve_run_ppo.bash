source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:./devel/lib

roscore &

catkin_make && argos3 -c src/TensorSwarm/argos_worlds/l_curve.argos &
argos3 -c src/TensorSwarm/argos_worlds/l_curve.argos &
argos3 -c src/TensorSwarm/argos_worlds/l_curve.argos &
argos3 -c src/TensorSwarm/argos_worlds/l_curve.argos &

python2 src/TensorSwarm/scripts/new/RunExperimentLCurve.py
