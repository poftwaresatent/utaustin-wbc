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

add_executable (tutsim tutsim.cpp)
target_link_libraries (tutsim opspace jspace_test yaml-cpp ${FLTK_LIBRARIES})
#set_target_properties (tutsim PROPERTIES LINK_FLAGS "-arch i386")
