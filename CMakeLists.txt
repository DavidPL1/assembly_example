cmake_minimum_required(VERSION 3.4)
project(assembly_example)

find_package(catkin REQUIRED)
catkin_package()

###########
## Build ##
###########


#############
## Install ##
#############

catkin_install_python(PROGRAMS nodes/assembly_example
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



