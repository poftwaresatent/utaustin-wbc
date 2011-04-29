include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../stanford_wbc/opspace/include
  /usr/realtime/include
  /home/meka/mekabot/m3/src
  ..
  )

add_definitions (-D__IN_RTAI__ -ffast-math)

add_executable (servo servo.cpp rt_util.cpp)
target_link_libraries (servo uta_opspace opspace jspace_test pthread yaml-cpp)

add_executable (torque_test torque_test.cpp rt_util.cpp)
target_link_libraries (torque_test jspace pthread)
