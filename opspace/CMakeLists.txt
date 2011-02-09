include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../reflexxes_otg/include
  ../yaml-cpp/include
  ../
  )

add_library (opspace SHARED
  opspace.cpp
  Task.cpp
  TaskFactory.cpp
  controller_library.cpp
  task_library.cpp
  )
target_link_libraries (opspace jspace reflexxes_otg yaml-cpp)

add_executable (hello_world hello_world.cpp)
target_link_libraries (hello_world opspace)

add_executable (testTask testTask.cpp)
target_link_libraries (testTask opspace jspace_test)

add_executable (testTaskFactory testTaskFactory.cpp)
target_link_libraries (testTaskFactory opspace)
