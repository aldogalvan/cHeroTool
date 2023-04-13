cmake_minimum_required(VERSION 3.25)
project(cHeroTool)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_ARCHITECTURES 61)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

#OpenGL
set(OpenGL_GL_PREFERENCE "GLVND")

#CHAI-3D
FIND_PACKAGE(CHAI3D REQUIRED CONFIG)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
    ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${CHAI3D_INCLUDE_DIRS})

FIND_PACKAGE(CHAI3D-GEL REQUIRED CONFIG)
ADD_DEFINITIONS(${CHAI3D-GEL_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
    ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D-GEL_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${CHAI3D-GEL_INCLUDE_DIRS})

FIND_PACKAGE(CHAI3D-BULLET REQUIRED CONFIG)
ADD_DEFINITIONS(${CHAI3D-BULLET_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
    ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D-BULLET_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${CHAI3D-BULLET_INCLUDE_DIRS})

# GLFW
if (EXISTS ${PROJECT_SOURCE_DIR}/extras/GLFW)
    add_subdirectory (${PROJECT_SOURCE_DIR}/extras/GLFW)
endif ()

#EIGEN3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -D__CUDACC__")

#Hierarchical Face Clustering
add_subdirectory(${PROJECT_SOURCE_DIR}/external/hierarchical-face-clustering)
include_directories(${PROJECT_SOURCE_DIR}/external/hierarchical-face-clustering)

# libigl
set(LIBIGL_ROOT_DIR "/home/agalvan-admin/libigl")
if(LIBIGL_ROOT_DIR)
    include_directories("${LIBIGL_ROOT_DIR}/include")
else()
    message(FATAL_ERROR "Cannot find libigl at LIBIGL_ROOT")
endif()

add_library(cHeroTool cHeroTool.cpp cHeroTool.h cOBB.h)

target_link_libraries(cHeroTool PUBLIC
        ${CHAI3D_LIBRARIES}
        ${CHAI3D-GEL_LIBRARIES}
        ${CHAI3D-BULLET_LIBRARIES}
        ${GLFW_LIBRARIES})

add_executable(example main.cpp)

target_link_libraries(example PUBLIC cHeroTool HierarchicalFaceClustering)

