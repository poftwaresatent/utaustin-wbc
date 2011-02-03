include_directories (
  ../stanford_wbc/jspace
  /usr/realtime/include
  /home/meka/mekabot/m3/src
  )

add_definitions (-D__IN_RTAI__ -ffast-math)

add_executable (m3_servo m3_servo.cpp)
target_link_libraries (m3_servo jspace_test pthread)
