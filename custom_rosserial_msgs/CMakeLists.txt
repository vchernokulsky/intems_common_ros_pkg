cmake_minimum_required(VERSION 3.5.1)
project(custom_rosserial_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roscpp
  std_msgs
)

add_message_files(FILES
  Log.msg
  TopicInfo.msg
)

add_service_files(FILES
  RequestParam.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS
  message_runtime
)
