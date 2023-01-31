# Quickstart Guide

This repository provides an example of how to write code to interface with our simulation environment.

For more information about interfacing the simulation with your own code or dockerizing your code, please visit our [wiki](https://github.com/DavidPL1/assembly_example/wiki).

# Building
Build the example images by running `./docker/build_screw.sh` first, and then `./docker/build_plug.sh`.

# Running

To start the simulation along with the example program, you can run `docker-compose -f ./docker/docker-compose.TASK.yml up`, replacing TASK with either screwing or plugging, depending on the task you want to launch.

As an example of how to record some data from within the docker container, we provide the watcher docker image (needs to be an extension of the assembly_server, in order to have access to the necessary ROS message definitions) which runs a rosbag recorder on the `/cameras/workspace_cam/rgb` topic.
In order for the data to persist, you need to create and mount a docker volume. As an example to this, have a look at the [docker-compose.rosbag.yml](docker/docker-compose.rosbag.yml) file.

To run any of the tasks and record the scene, run `docker-compose -f ./docker/docker-compose.TASK.yml -f ./docker/docker-compose.rosbag.yml`. Then wait some time and stop the containers with `C-c`. Now the docker directory should contain a `rosbag` subdirectory with a rosbag file in it.

You might have to run `rosbag fix` on the rosbag file if it has a `.active` extension (this can happen if the image is shut down forcefully before rosbag manages to shutdown gracefully).

On your host machine you can then play the rosbag and watch the camera stream with `rosrun rqt_image_view rqt_image_view /cameras/workspace_cam/rgb`.