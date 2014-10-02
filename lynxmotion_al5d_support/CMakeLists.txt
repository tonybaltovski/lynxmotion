cmake_minimum_required(VERSION 2.8.3)

project(lynxmotion_al5d_support)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir launch urdf)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
