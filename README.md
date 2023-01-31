# Quickstart Guide

This repository provides an example of how to write code to interface with our simulation environment.

For more information about interfacing the simulation with your own code or dockerizing your code, please visit our [wiki](https://github.com/DavidPL1/assembly_example/wiki).

# Building
Build the example images by running `./docker/build_screw.sh` first, and then `./docker/build_plug.sh`.

# Running

To start the simulation along with the example program, you can run `docker-compose -f ./docker/docker-compose.TASK.yml up`, replacing TASK with either screwing or plugging, depending on the task you want to launch.