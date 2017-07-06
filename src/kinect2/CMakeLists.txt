CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()
SET(PCL_DIR "/usr/local/share/pcl-1.8/PCLConfig.cmake")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share/pcl-1.8/PCLConfig.cmake")
IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()
SET(CMAKE_BUILD_TYPE "DEBUG")
PROJECT(kinect2)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS 
	     			roscpp 
                                std_msgs
                                geometry_msgs
                                message_generation
		        	libfreenect2)
find_package(PCL REQUIRED HINTS "/usr/local/share/pcl-1.8")
#find_package(OpenCV  REQUIRED )#HINTS /usr/local/share/OpenCV)
FIND_PACKAGE(CUDA REQUIRED)
find_package(OpenCV REQUIRED HINTS "/usr/local/share/OpenCV")

message("===============================${OpenCV_INCLUDE_DIRS}")

SET(MY_DIR ${kinect2_SOURCE_DIR})
OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR "${MY_DIR}/../libfreenect2")
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h
#message("===============================${freenect2_ROOT_DIR}")
FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
LIST(APPEND CMAKE_MODULE_PATH ${freenect2_ROOT_DIR}/cmake_modules) # FindGLFW3.cmake

IF(TARGET freenect2)
  MESSAGE(STATUS "Using in-tree freenect2 target")
  SET(freenect2_LIBRARIES freenect2)
  SET(freenect2_DLLS ${LIBFREENECT2_DLLS})
ELSE()
  FIND_PACKAGE(freenect2 REQUIRED)
  # Out-of-tree build will have to have DLLs manually copied.
ENDIF()

INCLUDE(FindCUDA)
INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

SET(Protonect_src
  Protonect.cpp
  Protonect.h
main.cpp
)

SET(Protonect_LIBRARIES
  ${freenect2_LIBRARIES}
)

SET(Protonect_DLLS
  ${freenect2_DLLS}
)


IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    LIST(APPEND Protonect_DLLS ${GLFW3_DLL})
    LIST(APPEND Protonect_src
      viewer.cpp
      ${flextGL_SOURCES}
    )
    LIST(APPEND Protonect_LIBRARIES
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )
    ADD_DEFINITIONS(-DEXAMPLES_WITH_OPENGL_SUPPORT=1)
  ENDIF()
ENDIF(ENABLE_OPENGL)

#generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(
   INCLUDE_DIRS .
#   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp libfreenect2 message_runtime
 #  DEPENDS opencv
)

#ADD_EXECUTABLE(Protonect
#  ${Protonect_src}
#)

add_library(kinect_ colorsegment.cpp LaterMethods.cpp
)
TARGET_LINK_LIBRARIES(kinect_
  ${OpenCV_LIBRARIES}
)
STRING(REPLACE "\\" "/" NVCUDASAMPLES_ROOT "$ENV{NVCUDASAMPLES_ROOT}")

CUDA_INCLUDE_DIRECTORIES(
"include"
"${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc"
"${NVCUDASAMPLES_ROOT}/common/inc"
)
CUDA_ADD_EXECUTABLE(kinect2 produce_pc_kernel.cu ${Protonect_src})

LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_50,code=sm_50)

TARGET_LINK_LIBRARIES(kinect2
  ${PCL_LIBRARIES}
  kinect_
 # ${OpenCV_LIBRARIES}
  ${Protonect_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#IF(WIN32)
#  INSTALL(TARGETS Protonect DESTINATION bin)
#  LIST(REMOVE_DUPLICATES Protonect_DLLS)
#  FOREACH(FILEI ${Protonect_DLLS})
#    ADD_CUSTOM_COMMAND(TARGET Protonect POST_BUILD
#      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FILEI} $<TARGET_FILE_DIR:Protonect>
#    )
#  ENDFOREACH(FILEI)
#  INSTALL(FILES ${Protonect_DLLS} DESTINATION bin)
#ENDIF()
