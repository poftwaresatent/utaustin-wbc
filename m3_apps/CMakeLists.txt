include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../opspace/include
  /usr/realtime/include
  /home/meka/mekabot/m3/src
  )

add_definitions (-D__IN_RTAI__ -ffast-math)

add_executable (servo servo.cpp rt_util.cpp)
target_link_libraries (servo opspace jspace_test pthread yaml-cpp)
