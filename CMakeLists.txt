cmake_minimum_required(VERSION 3.0.0)
project(comp-grafica VERSION 0.1.0)

include(CTest)
enable_testing()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(SDL2 REQUIRED)

add_executable(comp-grafica 

src/Utils/utils.cpp
src/Matrix/matrix.cpp
src/Scene/scene.cpp
src/LightSources/LightSource/lightSource.cpp
src/LightSources/Ambient/ambient.cpp
src/LightSources/Directional/directional.cpp
src/LightSources/Point/point.cpp
src/LightSources/Spot/spot.cpp
src/Objects/Object/object.cpp
src/Objects/Sphere/sphere.cpp
src/Objects/Plane/plane.cpp
src/Objects/Cylinder/cylinder.cpp
src/Objects/Cone/cone.cpp
src/Objects/Mesh/mesh.cpp
src/Canvas/canvas.cpp
src/main.cpp)

target_link_libraries(comp-grafica ${SDL2_LIBRARIES})
#target_link_libraries(comp-grafica SDL2::SDL2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a -pthread")
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)