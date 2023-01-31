# Since both tasks are built in the workspace, we only need to change the launch file of the screw example image
FROM s4dx/assembly_screw_example
SHELL ["/bin/bash", "-c"]

ENV BASE_DIR=/home
WORKDIR $BASE_DIR

ENTRYPOINT ["./assembly_entrypoint.sh", ""]
CMD ["--wait assembly_example assembly_plug.launch nonstop:=true"]
