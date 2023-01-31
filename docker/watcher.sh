#!/bin/bash

set -e

source /home/assembly_server/setup.bash

cd /data

exec $@
