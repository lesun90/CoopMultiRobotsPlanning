PROJECT(Runner CXX C)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

IF(COMMAND cmake_policy)
	CMAKE_POLICY(SET CMP0003 OLD)
	CMAKE_POLICY(SET CMP0005 OLD)
ENDIF(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

IF(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
ENDIF()

#############################################################################
#Compiler flags for normal (debug) and optimized (release) builds
#
#
#  SET(CMAKE_C_FLAGS, "-std=c++0x")
SET(CMAKE_CXX_FLAGS, "${CMAKE_CXX_FLAGS} -std=c++0x" CACHE STRING "Flags" FORCE)
MESSAGE("${CMAKE_CXX_FLAGS}")
IF(NOT MSVC_IDE)
  SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -std=c++0x" CACHE STRING
      "Flags used by the C++ compiler during debug builds." FORCE)
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -std=c++0x"
      CACHE STRING "Flags used by the C++ compiler during release builds." FORCE)
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -std=c++0x" CACHE STRING
      "Flags used by the C++ compiler during release builds with debug information." FORCE)

  SET(CMAKE_C_FLAGS_DEBUG "-g -O0" CACHE STRING
      "Flags used by the C compiler during debug builds." FORCE)
  SET(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING
      "Flags used by the C compiler during release builds." FORCE)
  SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g" CACHE STRING
      "Flags used by the C compiler during release builds with debug information." FORCE)
ENDIF(NOT MSVC_IDE)

IF(APPLE)
	LIST(APPEND CMAKE_SHARED_LINKER_FLAGS "-single_module")
ENDIF(APPLE)

#ADD_DEFINITIONS(-DDEBUG)

ADD_DEFINITIONS(-D_USE_MATH_DEFINES)

#############################################################################
#Find OpenGL and GLUT
#
INCLUDE(FindOpenGL)
IF(NOT OPENGL_INCLUDE_DIR)
  SET(OPENGL_INCLUDE_DIR GL CACHE STRING "OpenGL directory for header file" FORCE)
ENDIF(NOT OPENGL_INCLUDE_DIR)

INCLUDE(FindGLUT)
IF(NOT GLUT_INCLUDE_DIR)
  SET(GLUT_INCLUDE_DIR GL CACHE STRING "Glut header file" FORCE)
ENDIF(NOT GLUT_INCLUDE_DIR)
IF (NOT GLUT_glut_LIBRARY)
  IF (WIN32 AND NOT CYGWIN)
    SET(GLUT_glut_LIBRARY glut32 CACHE STRING "Glut library for win32" FORCE)
  ENDIF(WIN32 AND NOT CYGWIN)
ENDIF (NOT GLUT_glut_LIBRARY)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} /usr/local/include/bullet)

IF(GLUT_glut_LIBRARY)
  SET(INTERACTIVE_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})
ELSE (GLUT_glut_LIBRARY)
  SET(INTERACTIVE_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF(GLUT_glut_LIBRARY)

#FIND_LIBRARY(LAPACK_LIB lapack /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})
#IF(LAPACK_LIB)
#ADD_DEFINITIONS(-DHAS_LAPACK)
#ENDIF(LAPACK_LIB)

# Boost v1.49 or greater is required
set(BOOST_MIN_VERSION 1.49.0) # Version 1.49 or better is required
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS graph REQUIRED)


#############################################################################
INCLUDE_DIRECTORIES(src)
#INCLUDE_DIRECTORIES(src/AUV)
AUX_SOURCE_DIRECTORY(src/Utils SRC_UTILS)
AUX_SOURCE_DIRECTORY(src/Logic SRC_LOGIC)
AUX_SOURCE_DIRECTORY(src/External SRC_EXTERNAL)
AUX_SOURCE_DIRECTORY(src/External/PQP SRC_PQP)
AUX_SOURCE_DIRECTORY(src/External/PAS SRC_PAS)
AUX_SOURCE_DIRECTORY(src/MP SRC_MP)
AUX_SOURCE_DIRECTORY(src/Scenes SRC_SCENES)
AUX_SOURCE_DIRECTORY(src/Programs   SRC_PROGRAMS)
#AUX_SOURCE_DIRECTORY(src/AUV SRC_AUV)


ADD_EXECUTABLE(Runner
		${SRC_UTILS}
		${SRC_LOGIC}
		${SRC_EXTERNAL}
    ${SRC_PQP}
    ${SRC_PAS}
		${SRC_MP}
		${SRC_SCENES}
		${SRC_PROGRAMS}
	      )

FIND_LIBRARY(LIB_BT_DYNAMICS BulletDynamics /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})
FIND_LIBRARY(LIB_BT_COLLISION BulletCollision /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})
FIND_LIBRARY(LIB_BT_MATH LinearMath /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})
FIND_LIBRARY(PNG_LIB png /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})

MESSAGE("
*****************************************************************************
* EXTERNAL LIBRARIES
*   GL     : ${OPENGL_gl_LIBRARY}
*   GLU    : ${OPENGL_glu_LIBRARY}
*   GLUT   : ${GLUT_glut_LIBRARY}
*   BulletDynamics: ${LIB_BT_DYNAMICS}
*   BulletCollision: ${LIB_BT_COLLISION}
*   BulletMath: ${LIB_BT_MATH}
*   LAPACK : ${LAPACK_LIB}
*   CXX_FLAGS: ${CMAKE_CXX_FLAGS}
*****************************************************************************
")


TARGET_LINK_LIBRARIES(Runner
  ${INTERACTIVE_LIBS}
  ${LIB_BT_DYNAMICS}
  ${LIB_BT_COLLISION}
  ${LIB_BT_MATH}
  ${PNG_LIB}
  ${Boost_GRAPH_LIBRARY}
 # ${MOOS_LIBRARIES}
 # ${MOOSGeodesy_LIBRARIES}
 # ${MBUTIL}
 # ${CONTACTS}
 # ${GEOM}
  m
  pthread )



FIND_LIBRARY(DL_LIB dl /usr/lib/ /usr/local/lib/ ./lib/ ${LIBRARY_OUTPUT_PATH})
IF(DL_LIB)
   TARGET_LINK_LIBRARIES(Runner ${DL_LIB})
ENDIF(DL_LIB)
