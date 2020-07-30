cmake_minimum_required(VERSION 3.5.1)
project(custom_rosserial_python)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS nodes/message_info_service.py nodes/custom_serial_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
