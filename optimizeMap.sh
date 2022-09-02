#! /bin/bash

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.sh
rosservice call /optimize_map
