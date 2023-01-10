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

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



