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

add_executable (tutmain tutmain.cpp)
target_link_libraries (tutmain tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut1_coupling tut1_coupling.cpp)
target_link_libraries (tut1_coupling tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})

add_executable (tut02_jtask tut02_jtask.cpp)
target_link_libraries (tut02_jtask tut opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})
