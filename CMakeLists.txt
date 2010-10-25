cmake_minimum_required (VERSION 2.6.5)
if (COMMAND CMAKE_POLICY)
  # CMP0015: link_directories() treats paths relative to the source dir
  cmake_policy (SET CMP0015 NEW)
endif (COMMAND CMAKE_POLICY)

project (utaustin-wbc)
set (CMAKE_VERBOSE_MAKEFILE ON)

if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

##################################################
# OS detection

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    include_directories (
      /opt/local/include
      /opt/local/include/eigen2
      )
    link_directories (/opt/local/lib)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
    include_directories (/usr/include/eigen2)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      /usr/local/include/eigen2
      )
    link_directories (/usr/local/lib)
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################
# compiler flags (other than header and library paths)

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

##################################################
# hardcoded header and library paths will need to be replaced by
# configure-time detection code later.

include_directories (
  .
  ../stanford-wbc/tao
  ../stanford-wbc/jspace
  )

link_directories (
  ../stanford-wbc/build/tao
  ../stanford-wbc/build/jspace
  )

##################################################
# sources and targets for the UTAustin-WBC project

add_library (utaustin-wbc SHARED opspace/opspace.cpp)
target_link_libraries (utaustin-wbc jspace)

# add_executable (testOpspace testOpspace.cpp)
# target_link_libraries (testOpspace utaustin-wbc gtest)
