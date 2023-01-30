# Building
Build the image by running `build.sh`

# Running

The container sources the simulation ros environment and launches the simulation manager with default arguments.

## Dev Mode
This mode starts the simulation such that the host machine can connect to it, e.g. with rviz or any other component.

To run in dev mode run `docker run --rm --net=host -it s4dx/assembly_server`. Note: the `--net=host` option runs the container in the host network, meaning there are no restrictions with port communication (this is needed for ROS, since ROS nodes utilize a wide port range for communication). 
The default arguments to the manager (`verbose:=true headless:=true`) can be overridden by appending ros arguments directly to the docker run command. 

Other arguments that can be provided are:
TODO

## Pipeline Mode

1. clone/pull and build the dockerized assembly_example: `git clone git@gitlab.ub.uni-bielefeld.de:agni/grasplab/s4dx/assembly_example_dockerized.git && ./assembly_example_dockerized/build.sh`
2. Start everything automatically by running `docker-compose up`. This allows no kind of interaction from the host machine, you will only get the log outputs of the running containers.

### Train/Test Mode
TODO