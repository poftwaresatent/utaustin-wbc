include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../reflexxes_otg/include
  ../yaml-cpp/include
  ../opspace/include
  .
  )

add_library (uta_opspace SHARED
  ControllerNG.cpp
  HelloGoodbyeSkill.cpp
  )
target_link_libraries (uta_opspace opspace jspace reflexxes_otg yaml-cpp)
