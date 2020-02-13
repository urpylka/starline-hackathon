#!/bin/bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD

cd $ROOT_DIR

git subtree pull --prefix ../catkin_tws/src/kobuki_msg  https://github.com/yujinrobot/kobuki_msgs.git  kinetic  --squash -m  "auto-update subtree"

cd $EXEC_PATH
