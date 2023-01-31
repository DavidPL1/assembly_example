FROM s4dx/assembly_server
SHELL ["/bin/bash", "-c"]

ENV BASE_DIR=/home
WORKDIR $BASE_DIR
COPY watcher.sh .

ENTRYPOINT [ "/home/watcher.sh", "" ]
CMD ["rosbag record /cameras/workspace_cam/rgb"]
