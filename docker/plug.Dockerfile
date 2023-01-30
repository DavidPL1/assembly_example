FROM s4dx/assembly_screw_example
SHELL ["/bin/bash", "-c"]

ENV BASE_DIR=/home
WORKDIR $BASE_DIR

ENTRYPOINT ["./assembly_entrypoint.sh", ""]
CMD ["assembly_example assembly_plug.launch nonstop:=true"]
