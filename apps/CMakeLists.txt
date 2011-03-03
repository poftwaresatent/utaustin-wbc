include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../opspace/include
  /usr/realtime/include
  /home/meka/mekabot/m3/src
  )

add_definitions (-D__IN_RTAI__ -ffast-math)

add_executable (m3_servo m3_servo.cpp)
target_link_libraries (m3_servo jspace_test pthread)

add_executable (m3_tp_test m3_tp_test.cpp)
target_link_libraries (m3_tp_test opspace jspace_test pthread)

#add_executable (m3_behavior_test m3_behavior_test.cpp)
#target_link_libraries (m3_behavior_test opspace jspace_test pthread yaml-cpp)

add_executable (m3_jlimit_test m3_jlimit_test.cpp)
target_link_libraries (m3_jlimit_test opspace jspace_test pthread)
