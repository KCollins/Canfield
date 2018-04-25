cmake_minimum_required(VERSION 2.8.3)
project(canfield_joint_controller)
find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(canfield_joint_controller src/canfield_joint_controller.cpp)
cs_install()
cs_export()
