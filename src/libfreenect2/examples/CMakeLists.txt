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

PROJECT(libfreenect2_examples)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(PCL REQUIRED HINTS "/usr/local/share/pcl-1.8")

SET(MY_DIR ${libfreenect2_examples_SOURCE_DIR})
SET(DEPENDS_DIR "${MY_DIR}/../depends" CACHE STRING "Dependency directory")

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR ${MY_DIR}/..)
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

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
)

SET(Protonect_src
  Protonect.cpp
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

STRING(REPLACE "\\" "/" NVCUDASAMPLES_ROOT "$ENV{NVCUDASAMPLES_ROOT}")

CUDA_INCLUDE_DIRECTORIES(
"."
"${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc"
"${NVCUDASAMPLES_ROOT}/common/inc"
)
CUDA_ADD_EXECUTABLE(Protonect produce_pc_kernel.cu ${Protonect_src})

LIST(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_53,code=sm_53)

TARGET_LINK_LIBRARIES(Protonect
  ${PCL_LIBRARIES}
 # ${OpenCV_LIBRARIES}
  ${Protonect_LIBRARIES}
  ${CUDA_LIBRARIES}
 # ${catkin_LIBRARIES}
)

IF(WIN32)
  INSTALL(TARGETS Protonect DESTINATION bin)
  LIST(REMOVE_DUPLICATES Protonect_DLLS)
  FOREACH(FILEI ${Protonect_DLLS})
    ADD_CUSTOM_COMMAND(TARGET Protonect POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FILEI} $<TARGET_FILE_DIR:Protonect>
    )
  ENDFOREACH(FILEI)
  INSTALL(FILES ${Protonect_DLLS} DESTINATION bin)
ENDIF()
