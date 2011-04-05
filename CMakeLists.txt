cmake_minimum_required (VERSION 2.6.0)
if (COMMAND CMAKE_POLICY)
  # CMP0015: link_directories() treats paths relative to the source dir
  if(POLICY CMP0015)
    cmake_policy (SET CMP0015 NEW)
  endif(POLICY CMP0015)
endif (COMMAND CMAKE_POLICY)

project (utaustin-wbc)
set (CMAKE_VERBOSE_MAKEFILE ON)

if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

##################################################
# OS detection

if (WIN32)
  message ("--- detected Windows")
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    message ("--- detected OSX")
    add_definitions (-DOSX)
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      /opt/local/include/eigen2
      )
    include_directories (
      /opt/local/include
      /opt/local/include/eigen2
      )
    link_directories (/opt/local/lib)
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    message ("--- detected Linux")
    add_definitions (-DLINUX)
    list (APPEND CMAKE_REQUIRED_INCLUDES /usr/include/eigen2)
    include_directories (/usr/include/eigen2)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    message ("--- detected OpenBSD")
    add_definitions (-DOPENBSD)
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /usr/local/include
      /usr/local/include/eigen2
      )
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
# configure dependencies

include (CheckIncludeFileCXX)

# find Eigen2 (annoyingly, this is copy-pasted from the stanford_wbc subdirectory)

if (EIGEN2_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${EIGEN2_DIR}/include
    ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2
    ${EIGEN2_DIR}/eigen2
    )
  include_directories (
    ${EIGEN2_DIR}/include
    ${EIGEN2_DIR}
    ${EIGEN2_DIR}/include/eigen2
    ${EIGEN2_DIR}/eigen2
    )
endif (EIGEN2_DIR)
check_include_file_cxx (Eigen/Core HAVE_EIGEN2)
if (HAVE_EIGEN2)
  message ("*** found Eigen2 (required)")
else (HAVE_EIGEN2)
  message (FATAL_ERROR "Please install Eigen2, e.g. `sudo apt-get install libeigen2-dev' or from http://eigen.tuxfamily.org/ and pass -DEIGEN2_DIR:path=/path/to/eigen2 to cmake next time you call it.")
endif (HAVE_EIGEN2)

# find Boost (just for smart pointers though)

if (BOOST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
  include_directories (
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
endif (BOOST_DIR)
check_include_file_cxx (boost/shared_ptr.hpp HAVE_BOOST)
if (HAVE_BOOST)
  message ("*** found Boost (required)")
else (HAVE_BOOST)
  message (FATAL_ERROR "Please install the Boost smart_ptr library, and/or tell me where to find it by passing -DBOOST_DIR:path=/path/to/boost to cmake next time (you probably need to clear its cache first).")
endif (HAVE_BOOST)

# gtest (optional)

if (GTEST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES ${GTEST_DIR}/include ${GTEST_DIR})
  include_directories (${GTEST_DIR}/include ${GTEST_DIR})
  link_directories (${GTEST_DIR}/lib ${GTEST_DIR})
endif (GTEST_DIR)
check_include_file_cxx (gtest/gtest.h HAVE_GTEST_HEADER)
if (${HAVE_GTEST_HEADER})
  if (GTEST_DIR)
    find_library (HAVE_GTEST_LIB gtest PATHS ${GTEST_DIR} ${GTEST_DIR}/lib)
  else (GTEST_DIR)
    find_library (HAVE_GTEST_LIB gtest)
  endif (GTEST_DIR)
  if (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message (FATAL_ERROR "gtest library not found, although the header gtest/gtest.h was found")
  else (HAVE_GTEST_LIB MATCHES "NOTFOUND")
    message ("*** enabling gtest")
    set (HAVE_GTEST TRUE)
    add_definitions (-DHAVE_GTEST)
  endif (HAVE_GTEST_LIB MATCHES "NOTFOUND")
else (${HAVE_GTEST_HEADER})
  message ("--- gtest not found (optional)")
endif (${HAVE_GTEST_HEADER})

# detection of Meka / RTAI code

if (M3_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES ${M3_DIR}/src ${M3_DIR})
  include_directories (${M3_DIR}/src ${M3_DIR})
  ###  link_directories (${M3_DIR}/lib ${M3_DIR})
endif (M3_DIR)
check_include_file_cxx (m3/shared_mem/torque_shm_sds.h HAVE_M3_SHM_HEADER)
if (${HAVE_M3_SHM_HEADER})
  message ("*** enabling Meka / RTAI")
  set (HAVE_M3 TRUE)
  add_definitions (-DHAVE_M3)
else (${HAVE_M3_SHM_HEADER})
  message ("--- Meka / RTAI not found (optional)")
endif (${HAVE_M3_SHM_HEADER})

##################################################
# descend into subprojects
# NOTE: add new subprojects here!

subdirs (
  yaml-cpp
  stanford_wbc
  reflexxes_otg
  opspace
  )

if (HAVE_M3)
  subdirs (m3_apps)
endif (HAVE_M3)
