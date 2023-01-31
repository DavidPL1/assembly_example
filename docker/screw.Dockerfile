FROM s4dx/assembly_server as base
# Use bash as executing shell for RUN commands
SHELL ["/bin/bash", "-c"]

ENV BASE_DIR=/home
WORKDIR $BASE_DIR

# Setup workspace to extend the server workspace and set install path
RUN mkdir -p catkin_ws/src && \
    mkdir user_ws && \
    source assembly_server/setup.bash && \
    cd catkin_ws && \
    catkin init && \
    catkin config --extend ${BASE_DIR}/assembly_server --install

# Copy ros sources into workspace
COPY assembly_example_ros catkin_ws/src/assembly_example

# Install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths catkin_ws/src --ignore-src --rosdistro ${ROS_DISTRO} && \
    rm -rf /var/lib/apt/lists/*

# Build in separate stage to later discard all unnecessary build artifacts without an extra layer
FROM base as builder

WORKDIR $BASE_DIR/catkin_ws

# Build catkin ws
RUN catkin build -DCMAKE_BUILD_TYPE=Release && \
    rm -rf ${BASE_DIR}/catkin_ws/src

# Start from base stage
FROM base as production
WORKDIR $BASE_DIR

# Copy Deployed Workspace from build stage
COPY --from=builder $BASE_DIR/catkin_ws/install user_ws/

# Add entrypoint script
COPY assembly_entrypoint.sh .

# Configure entrypoint to run the script
ENTRYPOINT ["./assembly_entrypoint.sh", ""]

# Set default arguments for entrypoint script
CMD ["--wait assembly_example assembly_screw.launch"]
