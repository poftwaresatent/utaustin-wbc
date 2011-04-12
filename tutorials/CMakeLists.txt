include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../opspace/include
  )

set (FLTK_SKIP_OPENGL ON)
set (FLTK_SKIP_FORMS ON)
set (FLTK_SKIP_IMAGES ON)
set (FLTK_SKIP_FLUID ON)
include (FindFLTK)

add_library (tut STATIC tutsim.cpp)

add_executable (tut00_test tut00_test.cpp)
target_link_libraries (tut00_test tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut01_joint_coupling tut01_joint_coupling.cpp)
target_link_libraries (tut01_joint_coupling tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut02_jtask tut02_jtask.cpp)
target_link_libraries (tut02_jtask tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})
