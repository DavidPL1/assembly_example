FROM s4dx/assembly_server as base
SHELL ["/bin/bash", "-c"]

ENV BASE_DIR=/home
WORKDIR $BASE_DIR

RUN mkdir -p catkin_ws/src && \
    mkdir user_ws && \
    source assembly_server/setup.bash && \
    cd catkin_ws && \
    catkin init && \
    catkin config --extend ${BASE_DIR}/assembly_server --install

COPY assembly_example_ros catkin_ws/src/assembly_example
# Install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths catkin_ws/src --ignore-src --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

FROM base as builder

WORKDIR $BASE_DIR/catkin_ws

# Build catkin ws
RUN catkin build -DCMAKE_BUILD_TYPE=Release && \
    rm -rf ${BASE_DIR}/catkin_ws/src

FROM base as production
WORKDIR $BASE_DIR
# Copy Deployed Workspace
COPY --from=builder $BASE_DIR/catkin_ws/install user_ws/

RUN sudo apt install -y ros-noetic-moveit-planners-chomp ros-noetic-moveit-planners

COPY assembly_entrypoint.sh .
# ENTRYPOINT ["/bin/bash"]
ENTRYPOINT ["./assembly_entrypoint.sh", ""]
CMD ["--wait assembly_example assembly_screw.launch"]
