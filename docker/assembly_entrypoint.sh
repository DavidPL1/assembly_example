#!/bin/bash

set -e

source /home/user_ws/setup.bash

# echo $(ls $(rospack find assembly_manager)/launch)

echo "roslaunch $@"
exec roslaunch $@
