#! /bin/bash

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.sh
roslaunch ${SEED_HOME}/src/matching.launch

