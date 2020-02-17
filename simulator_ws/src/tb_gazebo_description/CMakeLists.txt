cmake_minimum_required(VERSION 2.8.3)

project(tb_gazebo_description)
find_package(catkin REQUIRED COMPONENTS urdf xacro)

catkin_package(
   CATKIN_DEPENDS urdf xacro
)
