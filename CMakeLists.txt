cmake_minimum_required(VERSION 3.0.2)
project(phidget_ros_drivers)

find_package(catkin REQUIRED)

catkin_python_setup() #call setup py 

# if there are new msgs 
#generate_messages()

catkin_package(
)

