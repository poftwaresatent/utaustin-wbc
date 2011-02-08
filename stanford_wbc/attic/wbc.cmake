cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)


##################################################
#
# wbc_getvars ()
#
# Check for necessary variables and set them if possible.
# - 3rdparty (some optional) dependencies:
#   - GTEST_DIR
#   - XMLRPC_DIR
#   - LOG4CXX_DIR
#   - EXPAT_DIR
#   - NETWRAP_DIR
#   These variables, if not set, will be attempted to be taken from
#   the environment. As a side effect, include and link directories
#   will be adjusted to find gtest, xmlrpc++, log4cxx, expat, and
#   libnetwrapper headers and libraries.
# - if LOG4CXX_DIR==DISABLED, then it explicitly sets the
#   DISABLE_LOGGING preprocessor symbol
# - in addition, ROS_BINDEPS_PATH is used (if available) to provide
#   LOG4CXX_DIR, if that is still undefined after the pther checks.
#
macro (wbc_getvars)
  # try to get GTEST_DIR from CMake or environment
  if (NOT GTEST_DIR)
    set (GTEST_DIR $ENV{GTEST_DIR})
  endif (NOT GTEST_DIR)
  if (GTEST_DIR)
    message ("[WBC] GTEST_DIR is set to ${GTEST_DIR}")
    list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
    include_directories (${GTEST_DIR}/include ${GTEST_DIR})
    link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
  endif (GTEST_DIR)
  
  # try to get XMLRPC_DIR from CMake or environment
  if (NOT XMLRPC_DIR)
    set (XMLRPC_DIR $ENV{XMLRPC_DIR})
  endif (NOT XMLRPC_DIR)
  if (XMLRPC_DIR)
    message ("[WBC] XMLRPC_DIR is set to ${XMLRPC_DIR}")
    list (APPEND CMAKE_REQUIRED_INCLUDES ${XMLRPC_DIR}/include ${XMLRPC_DIR})
    include_directories (${XMLRPC_DIR}/include ${XMLRPC_DIR})
    link_directories (${XMLRPC_DIR}/lib ${XMLRPC_DIR})
  endif (XMLRPC_DIR)
  
  # try to get EXPAT_DIR from CMake or environment
  if (NOT EXPAT_DIR)
    set (EXPAT_DIR $ENV{EXPAT_DIR})
  endif (NOT EXPAT_DIR)
  if (EXPAT_DIR)
    message ("[WBC] EXPAT_DIR is set to ${EXPAT_DIR}")
    list (APPEND CMAKE_REQUIRED_INCLUDES ${EXPAT_DIR}/include ${EXPAT_DIR})
    include_directories (${EXPAT_DIR}/include ${EXPAT_DIR})
    link_directories (${EXPAT_DIR}/lib ${EXPAT_DIR})
  endif (EXPAT_DIR)
  
  # try to get NETWRAP_DIR from CMake or environment
  if (NOT NETWRAP_DIR)
    set (NETWRAP_DIR $ENV{NETWRAP_DIR})
  endif (NOT NETWRAP_DIR)
  if (NETWRAP_DIR)
    message ("[WBC] NETWRAP_DIR is set to ${NETWRAP_DIR}")
    list (APPEND CMAKE_REQUIRED_INCLUDES ${NETWRAP_DIR}/include ${NETWRAP_DIR})
    include_directories (${NETWRAP_DIR}/include ${NETWRAP_DIR})
    link_directories (${NETWRAP_DIR}/lib ${NETWRAP_DIR})
  endif (NETWRAP_DIR)
  
  # try to get LOG4CXX_DIR from CMake or environment
  if (NOT LOG4CXX_DIR)
    set (LOG4CXX_DIR $ENV{LOG4CXX_DIR})
  endif (NOT LOG4CXX_DIR)
  if (NOT LOG4CXX_DIR)
    set (LOG4CXX_DIR $ENV{ROS_BINDEPS_PATH})
  endif (NOT LOG4CXX_DIR)
  if (LOG4CXX_DIR)
    if (LOG4CXX_DIR STREQUAL "DISABLED")
      message ("[WBC] logging has been explicitly disabled")
      add_definitions (-DDISABLE_LOGGING)
    else (LOG4CXX_DIR STREQUAL "DISABLED")
      message ("[WBC] LOG4CXX_DIR is set to ${LOG4CXX_DIR}")
      list (APPEND CMAKE_REQUIRED_INCLUDES ${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
      include_directories (${LOG4CXX_DIR}/include ${LOG4CXX_DIR})
      link_directories (${LOG4CXX_DIR}/lib ${LOG4CXX_DIR})
    endif (LOG4CXX_DIR STREQUAL "DISABLED")
  endif (LOG4CXX_DIR)
endmacro (wbc_getvars)


##################################################
#
# wbc_init (project_name)
#
# Set up some sensible flags and variables. Calls CMake's project()
# command for you, using the provided project_name parameter.
# - CMAKE_VERBOSE_MAKEFILE will be switched on
# - BUILD_SHARED_LIBS will be switched on, unless disabled before
# - CMAKE_BUILD_TYPE will be set to "Debug" unless defined to something else before
# - operating-system flag -DWIN32, -DOSX, -DLINUX, or -DOPENBSD will be set
# - compiler flags -pipe and -Wall will be set
# - compiler flag -O0 will be set for debug build
# - if desired (by setting ANSI=true), also add -ansi and -pedantic compiler flags
# - if desired (by setting COVERAGE=true), also add -fprofile-arcs and -ftest-coverage compiler flags
# - preprocessor symbol TIXML_USE_STL will be set, because that seems
#   a good idea and if we mix in our optional ROS support, we have to
#   (there is no end to headaches due to tinyxml version and
#   compilation issues, grr)
# - CMake variables and preprocessor definitions to detect some
#   optional 3rdparty modules:
#   - HAVE_GTEST
#   - HAVE_XMLRPC
#   - HAVE_LOG4CXX
#     note that if the LOG4CXX_DIR variable is set to DISABLED, then
#     DISABLE_LOGGING will be set regardless of whether log4cxx is
#     installed on your system.
#   - HAVE_CURSES
#   - HAVE_EXPAT
#   - HAVE_NETWRAP
#
macro (wbc_init PROJECT_NAME)
  message ("[WBC] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
  message ("[WBC] BEGIN base config of ${PROJECT_NAME}")
  
  project (${PROJECT_NAME})
  
  add_definitions (-DTIXML_USE_STL)
  
  # nothing worse than not knowing why a header could not be found, so
  # we wanna see those -I directives etc
  set (CMAKE_VERBOSE_MAKEFILE ON)
  if (NOT BUILD_SHARED_LIBS)
    set (BUILD_SHARED_LIBS True)
  endif (NOT BUILD_SHARED_LIBS)
  
  # some things might depend on the operating system
  if (WIN32)
    message ("[WBC] Detected Microsoft Windows")
    add_definitions (-DWIN32)
  else (WIN32)
    if (APPLE)
      message ("[WBC] Detected Mac OS X")
      add_definitions (-DOSX)
      # add the default macports location to include and link path
      include_directories (/opt/local/include)
      link_directories (/opt/local/lib)
    endif (APPLE)
    if (CMAKE_SYSTEM_NAME MATCHES Linux)
      message ("[WBC] Detected Linux")
      add_definitions (-DLINUX)
    endif (CMAKE_SYSTEM_NAME MATCHES Linux)
    if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
      message ("[WBC] Detected OpenBSD")
      add_definitions (-DOPENBSD)
      include_directories (/usr/local/include)
      link_directories (/usr/local/lib)
    endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
  endif(WIN32)
  
  # -pipe and -Wall are pretty sensible default flags for GCC
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
  if (CXX_FLAG_pipe)
    add_definitions (-pipe)
  endif (CXX_FLAG_pipe)
  check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
  if (CXX_FLAG_Wall)
    add_definitions (-Wall)
  endif (CXX_FLAG_Wall)
  if (ANSI)
    check_cxx_compiler_flag (-ansi CXX_FLAG_ansi)
    if (CXX_FLAG_ansi)
      add_definitions (-ansi)
    else (CXX_FLAG_ansi)
      message ("[WBC] oops, ANSI requested but compiler does not understand -ansi")
    endif (CXX_FLAG_ansi)
    check_cxx_compiler_flag (-pedantic CXX_FLAG_pedantic)
    if (CXX_FLAG_pedantic)
      add_definitions (-pedantic)
    else (CXX_FLAG_pedantic)
      message ("[WBC] oops, ANSI requested but compiler does not understand -pedantic")
    endif (CXX_FLAG_pedantic)
  endif (ANSI)
  if (COVERAGE)
    check_cxx_compiler_flag (-fprofile-arcs CXX_FLAG_fprofile_arcs)
## broken? my g++ accepts it, but cmake does not think so    if (CXX_FLAG_fprofile_arcs)
      add_definitions (-fprofile-arcs)
##    else (CXX_FLAG_fprofile_arcs)
##      message ("[WBC] oops, COVERAGE requested but compiler does not understand -fprofile-arcs")
##    endif (CXX_FLAG_fprofile_arcs)
    check_cxx_compiler_flag (-ftest-coverage CXX_FLAG_ftest_coverage)
    if (CXX_FLAG_ftest_coverage)
      add_definitions (-ftest-coverage)
    else (CXX_FLAG_ftest_coverage)
      message ("[WBC] oops, COVERAGE requested but compiler does not understand -ftest-coverage")
    endif (CXX_FLAG_ftest_coverage)
    set (MAYBE_GCOV gcov)
  endif (COVERAGE)

  # explicitly default to debug build and set -O0 in that case
  if (NOT CMAKE_BUILD_TYPE)
    SET (CMAKE_BUILD_TYPE Debug)
  endif (NOT CMAKE_BUILD_TYPE)
  if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
    if (CXX_FLAG_O0)
      add_definitions (-O0)
    endif (CXX_FLAG_O0)
  endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  
  # make sure we can find WBC headers and libraries
  wbc_getvars()
  
  if (NOT WBC_ROOT)
    set (WBC_ROOT $ENV{WBC_ROOT})
  endif (NOT WBC_ROOT)
  if (WBC_ROOT)
    message ("[WBC] WBC_ROOT is set to ${WBC_ROOT}")
    list (APPEND CMAKE_REQUIRED_INCLUDES ${WBC_ROOT}/include ${WBC_ROOT})
    include_directories (${WBC_ROOT}/include ${WBC_ROOT})
    link_directories (${WBC_ROOT}/lib ${WBC_ROOT})
  else (WBC_ROOT)
    message ("[WBC] WBC_ROOT is not set (this is probably harmless)")
  endif (WBC_ROOT)
  
  # try to find 3rdparty stuff
  include (CheckIncludeFileCXX)
  
  if (LOG4CXX_DIR STREQUAL "DISABLED")
    message ("[WBC] logging has been explicitly disabled")
    add_definitions (-DDISABLE_LOGGING)
  else (LOG4CXX_DIR STREQUAL "DISABLED")
    check_include_file_cxx (log4cxx/logger.h HAVE_LOG4CXX)
    if (HAVE_LOG4CXX)
      message ("[WBC] found log4cxx headers")
      add_definitions (-DHAVE_LOG4CXX)
      list (APPEND LIBS log4cxx)
    else (HAVE_LOG4CXX)
      message ("[WBC] WARNING did not find log4cxx, will use simplistic logging")
    endif (HAVE_LOG4CXX)
  endif (LOG4CXX_DIR STREQUAL "DISABLED")
  
  check_include_file_cxx (curses.h HAVE_CURSES)
  if (${HAVE_CURSES})
    message ("[WBC] found curses headers")
    add_definitions (-DHAVE_CURSES)
  else (${HAVE_CURSES})
    message ("[WBC] WARNING did not find curses, key codes will not be available")
  endif (${HAVE_CURSES})
  
  check_include_file_cxx (gtest/gtest.h HAVE_GTEST_HEADER)
  if (${HAVE_GTEST_HEADER})
    message ("[WBC] found gtest headers")
    if (GTEST_DIR)
      find_library (HAVE_GTEST_LIB gtest PATHS ${GTEST_DIR} ${GTEST_DIR}/lib)
    else (GTEST_DIR)
      find_library (HAVE_GTEST_LIB gtest)
    endif (GTEST_DIR)
    if (HAVE_GTEST_LIB MATCHES "NOTFOUND")
      message (FATAL_ERROR "gtest library not found, although the header gtest/gtest.h was found")
    else (HAVE_GTEST_LIB MATCHES "NOTFOUND")
      message ("[WBC] found gtest library")
      set (HAVE_GTEST TRUE)
      add_definitions (-DHAVE_GTEST)
    endif (HAVE_GTEST_LIB MATCHES "NOTFOUND")
  else (${HAVE_GTEST_HEADER})
    message ("[WBC] WARNING did not find gtest, some tests will not be available")
  endif (${HAVE_GTEST_HEADER})
  
  check_include_file_cxx (XmlRpc.h HAVE_XMLRPC_HEADER)
  if (${HAVE_XMLRPC_HEADER})
    message ("[WBC] found XmlRpc++ headers")
    if (XMLRPC_DIR)
      find_library (HAVE_XMLRPC_LIB XmlRpc PATHS ${XMLRPC_DIR} ${XMLRPC_DIR}/lib)
    else (XMLRPC_DIR)
      find_library (HAVE_XMLRPC_LIB XmlRpc)
    endif (XMLRPC_DIR)
    if (HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
      message (FATAL_ERROR "XmlRpc++ library not found, although the header XmlRpc.h was found")
    else (HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
      message ("[WBC] found XmlRpc++ library")
      add_definitions (-DHAVE_XMLRPC)
      set (HAVE_XMLRPC TRUE)
    endif (HAVE_XMLRPC_LIB MATCHES "NOTFOUND")
  else (${HAVE_XMLRPC_HEADER})
    message ("[WBC] WARNING did not find XmlRpc++, some bindings will not be available")
  endif (${HAVE_XMLRPC_HEADER})
  
  check_include_file_cxx (expat.h HAVE_EXPAT_HEADER)
  if (${HAVE_EXPAT_HEADER})
    message ("[WBC] found expat headers")
    if (EXPAT_DIR)
      find_library (HAVE_EXPAT_LIB expat PATHS ${EXPAT_DIR} ${EXPAT_DIR}/lib)
    else (EXPAT_DIR)
      find_library (HAVE_EXPAT_LIB expat)
    endif (EXPAT_DIR)
    if (HAVE_EXPAT_LIB MATCHES "NOTFOUND")
      message (FATAL_ERROR "expat library not found, although the header expat.h was found")
    else (HAVE_EXPAT_LIB MATCHES "NOTFOUND")
      message ("[WBC] found expat library")
      add_definitions (-DHAVE_EXPAT)
      set (HAVE_EXPAT TRUE)
    endif (HAVE_EXPAT_LIB MATCHES "NOTFOUND")
  else (${HAVE_EXPAT_HEADER})
    message ("[WBC] WARNING did not find expat, some things will not be available")
  endif (${HAVE_EXPAT_HEADER})
  
  check_include_file_cxx (NetWrapper.h HAVE_NETWRAP_HEADER)
  if (${HAVE_NETWRAP_HEADER})
    message ("[WBC] found netwrap headers")
    if (NETWRAP_DIR)
      find_library (HAVE_NETWRAP_LIB netwrapper PATHS ${NETWRAP_DIR} ${NETWRAP_DIR}/lib)
    else (NETWRAP_DIR)
      find_library (HAVE_NETWRAP_LIB netwrapper)
    endif (NETWRAP_DIR)
    if (HAVE_NETWRAP_LIB MATCHES "NOTFOUND")
      message (FATAL_ERROR "netwrap library not found, although the header NetWrapper.h was found")
    else (HAVE_NETWRAP_LIB MATCHES "NOTFOUND")
      message ("[WBC] found netwrap library")
      add_definitions (-DHAVE_NETWRAP)
      set (HAVE_NETWRAP TRUE)
    endif (HAVE_NETWRAP_LIB MATCHES "NOTFOUND")
  else (${HAVE_NETWRAP_HEADER})
    message ("[WBC] WARNING did not find libnetwrapper, some things will not be available")
  endif (${HAVE_NETWRAP_HEADER})
  
  message ("[WBC] FINISHED base config of ${PROJECT_NAME}")
  message ("[WBC] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
endmacro (wbc_init)


##################################################
#
# wbc_find_eigen2()
#
# Try to find Eigen version 2.x (see http://eigen.tuxfamily.org/)
#
# Use the EIGEN2_DIR variable to point this macro to a custom
# install location.
#
macro (wbc_find_eigen2)
  # NOTE: the eigen2/ prefixing is a quick hack due to CMake
  # quirks. We should actually explicit check for the
  # eigen2/Eigen/COre header, and only then tweak the include
  # path... but CMake does not seem to tell us where exactly it found
  # a header, so we'd have to jump through some hoops via find_path()
  # or so.
  if (NOT EIGEN2_DIR)
    set (EIGEN2_DIR $ENV{EIGEN2_DIR})
  endif (NOT EIGEN2_DIR)
  if (EIGEN2_DIR)
    message ("[WBC] EIGEN2_DIR is set to ${EIGEN2_DIR}")
    list (APPEND CMAKE_REQUIRED_INCLUDES ${EIGEN2_DIR}/include ${EIGEN2_DIR} ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
    include_directories (${EIGEN2_DIR}/include ${EIGEN2_DIR} ${EIGEN2_DIR}/include/eigen2 ${EIGEN2_DIR}/eigen2)
  else (EIGEN2_DIR)
    # This is the standard install location under Debian, maybe others
    # as well. See not on quick hack above.
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
    include_directories (/usr/include/eigen2)
  endif (EIGEN2_DIR)
  
  check_include_file_cxx (Eigen/Core HAVE_EIGEN2)
  if (${HAVE_EIGEN2})
    message ("[WBC] found Eigen headers")
  else (${HAVE_EIGEN2})
    message (FATAL_ERROR "[WBC] Eigen2 library not found. Please install it e.g. using `sudo apt-get install libeigen2-dev' or from http://eigen.tuxfamily.org/ and tell me where to find it by passing the -DEIGEN2_DIR:path=/path/to/eigen2")
  endif (${HAVE_EIGEN2})
endmacro (wbc_find_eigen2)


#ATTIC# ##################################################
#ATTIC# #
#ATTIC# # wbc_find_urdf()
#ATTIC# #
#ATTIC# # Try to find ROS. If found, try to find URDF. If everything works as
#ATTIC# # desired, it sets up the include and link directives for inclusion of
#ATTIC# # the URDF package. The following variables will be set if URDF was
#ATTIC# # found:
#ATTIC# # - HAVE_ROS=true
#ATTIC# # - HAVE_URDF=true
#ATTIC# # - URDF_INCLUDES the directories to include
#ATTIC# # - URDF_CFLAGS other CFLAGS required to build the URDF
#ATTIC# # - URDF_LIBDIRS the directories to add to the linker search path
#ATTIC# # - URDF_LIBS the libraries to link with
#ATTIC# #
#ATTIC# # You can set TRY_ROS=false on the CMake command line in order to skip
#ATTIC# # all of this.
#ATTIC# #
#ATTIC# macro (wbc_find_urdf)
#ATTIC#   
#ATTIC#   if (NOT TRY_ROS)
#ATTIC#     set (TRY_ROS true)
#ATTIC#   endif (NOT TRY_ROS)
#ATTIC#   
#ATTIC#   if (TRY_ROS)
#ATTIC#     if (NOT $ENV{ROS_ROOT} STREQUAL "")
#ATTIC#       if (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#ATTIC# 	message ("[WBC] WARNING ROS_ROOT is set but rosbuild.cmake not found")
#ATTIC# 	set (HAVE_ROS-NOTFOUND)
#ATTIC#       else (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#ATTIC# 	set (HAVE_ROS true)
#ATTIC# 	include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#ATTIC# 	rosbuild_find_ros_package (urdf)
#ATTIC# 	if (${urdf_PACKAGE_PATH} STREQUAL "")
#ATTIC# 	  message (FATAL_ERROR "ROS support enabled but urdf package not found, try running 'rosmake urdf'")
#ATTIC# 	  set (HAVE_URDF-NOTFOUND)
#ATTIC# 	else (${urdf_PACKAGE_PATH} STREQUAL "")
#ATTIC# 	  message ("[WBC] enabling ROS support for URDF")
#ATTIC# 	  set (HAVE_URDF true)
#ATTIC# 	  rosbuild_invoke_rospack (urdf URDF INCLUDES cflags-only-I)
#ATTIC# 	  include_directories (${URDF_INCLUDES})
#ATTIC# #	  message ("[WBC] URDF_INCLUDES is ${URDF_INCLUDES}")
#ATTIC# 	  rosbuild_invoke_rospack (urdf URDF OTHER_CFLAGS cflags-only-other)
#ATTIC# 	  add_definitions (${URDF_CFLAGS})
#ATTIC# #	  message ("[WBC] URDF_CFLAGS is ${URDF_CFLAGS}")
#ATTIC# 	  rosbuild_invoke_rospack (urdf URDF LIBDIRS libs-only-L)
#ATTIC# #	  message ("[WBC] URDF_LIBDIRS is ${URDF_LIBDIRS}")
#ATTIC# 	  link_directories (${URDF_LIBDIRS})
#ATTIC# 	  rosbuild_invoke_rospack (urdf URDF LIBS libs-only-l)
#ATTIC# 	endif (${urdf_PACKAGE_PATH} STREQUAL "")
#ATTIC#       endif (NOT EXISTS $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#ATTIC#     endif (NOT $ENV{ROS_ROOT} STREQUAL "")
#ATTIC#     if (NOT HAVE_ROS)
#ATTIC#       message ("ROS support disabled, will skip URDF conversion stuff")
#ATTIC#     endif (NOT HAVE_ROS)
#ATTIC#   endif (TRY_ROS)
#ATTIC#   
#ATTIC# endmacro (wbc_find_urdf)


##################################################
#
# wbc_add_plugin (name source1 source2 ... sourceN)
#
# Add a plugin to the whole-body controller. This is basically just a
# wrapper around CMake's standard add_library() command. The plugin
# file will be named according to the CMake rules concerning the given
# name, and the sources passed to the wbc_add_plugin() command will be
# compiled and linked into the plugin.
#
# The plugin can be stored in various locations, depending on your
# setup:
# - if the CMake variable WBC_PLUGIN_PATH is set, the plugin's .so
#   file gets written there
# - otherwise, if WBC_ROOT is set in CMake or the environment, it gets
#   written to ${WBC_ROOT}/plugins
# - otherwise, it ends up in the CMake default location for libraries
#
macro (wbc_add_plugin PLUGIN_NAME)
  message ("[WBC] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
  message ("[WBC] BEGIN configuring plugin ${PLUGIN_NAME}")
  
  wbc_init (${PLUGIN_NAME})
  
  add_library (${PLUGIN_NAME} MODULE ${ARGN})
  
  message ("[WBC] linking plugin ${PLUGIN_NAME} with MAYBE_GCOV which is `${MAYBE_GCOV}'...")
  target_link_libraries (${PLUGIN_NAME} ${MAYBE_GCOV})
  
  set (PLUGIN_OUT_DIR ${WBC_PLUGIN_PATH})
  if (NOT PLUGIN_OUT_DIR)
    # Note that WBC_ROOT can come from the environment, this is done
    # in the wbc_init() macro which we call a couple of lines above.
    if (WBC_ROOT)
      set (PLUGIN_OUT_DIR ${WBC_ROOT}/plugins)
    endif (WBC_ROOT)
  endif (NOT PLUGIN_OUT_DIR)
  
  if (PLUGIN_OUT_DIR)
    message ("[WBC] setting plugin output path to ${PLUGIN_OUT_DIR}")
    set_target_properties (${PLUGIN_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PLUGIN_OUT_DIR})
  else (PLUGIN_OUT_DIR)
    message ("[WBC] falling back on the CMake default plugin output path")
  endif (PLUGIN_OUT_DIR)
  
  install (TARGETS ${PLUGIN_NAME} DESTINATION plugins)
  
  message ("[WBC] FINISHED configuring WBC plugin ${PLUGIN_NAME}")
  message ("[WBC] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
endmacro (wbc_add_plugin)


##################################################
#
# wbc_add_executable()
#
# Just like add_executable(), but adds linker directives to find the
# Stanford_WBC libraries and other dependencies.
#
macro (wbc_add_executable EXECUTABLE_NAME)
  add_executable (${EXECUTABLE_NAME} ${ARGN})
  target_link_libraries (${EXECUTABLE_NAME} Stanford_WBC ${MAYBE_GCOV})
  set_target_properties (${EXECUTABLE_NAME} PROPERTIES INSTALL_RPATH "${URDF_LIBDIRS};${CMAKE_INSTALL_PREFIX}/lib")
endmacro (wbc_add_executable)
