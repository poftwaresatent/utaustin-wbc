include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../
  /usr/realtime/include
  /home/meka/mekabot/m3/src
  )

add_definitions (-D__IN_RTAI__ -ffast-math)

add_executable (m3_servo m3_servo.cpp)
target_link_libraries (m3_servo jspace_test pthread)

add_executable (m3_tp_test m3_tp_test.cpp)
target_link_libraries (m3_tp_test opspace jspace_test pthread)
