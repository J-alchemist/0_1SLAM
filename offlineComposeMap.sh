#! /bin/bash

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.sh
rosrun nonlinear_optimize offline_compose_map 
