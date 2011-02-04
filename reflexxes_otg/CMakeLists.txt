cmake_minimum_required (VERSION 2.4.6)

if (EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  message ("*** enabling ROS support")
  include (ROS-CMakeLists.txt)
else (EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  message ("--- did not find ROS, using subproject CMake setup")
  include_directories (include)
  add_library (reflexxes_otg
    src/TypeIOTG.cpp
    src/TypeIOTGDecision.cpp
    src/TypeIOTGMath.cpp
    src/TypeIOTGPolynomial.cpp
    src/TypeIOTGProfiles.cpp
    )
  add_executable (trjgen src/trjgen.cpp)
  target_link_libraries (trjgen reflexxes_otg)
endif (EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
