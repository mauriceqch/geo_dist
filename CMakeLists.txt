cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PROJECT_OUTPUT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../test")
set(CMAKE_CONFIGURATION_TYPES "Release;Debug" CACHE STRING "Release;Debug")

project(pc_error)

find_package(PCL 1.8 COMPONENTS common octree io search features REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found.")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.61.0 COMPONENTS program_options)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
#  target_link_libraries(progname ${Boost_LIBRARIES})
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

file(GLOB_RECURSE PROJ_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h*)
file(GLOB SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/./")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES ${VTK_LIBRARIES} ${QHULL_LIBRARIES} ${OPENNI_LIBRARIES} ${OPENNI2_LIBRARIES}) # no need for vtk, qhull, openni/openni2

message(STATUS "PCL_LIBRARIES=${PCL_LIBRARIES}")

if (MSVC)
  message(STATUS "TO BE TESTED")
  # add_definitions("/D\"_CRT_SECURE_NO_WARNINGS\"")
  # add_definitions("/D\"_SCL_SECURE_NO_WARNINGS\"")
  add_executable(pc_error ${SOURCE})
  # add_executable(pc_error main.cpp.cpp stdafx.cpp ${PROJ_HEADERS})
else()
  list(REMOVE_ITEM SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/stdafx.cpp")
  add_executable(pc_error ${SOURCE})
endif()

target_link_libraries(pc_error ${PCL_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY})
install(TARGETS pc_error DESTINATION ".")

set_target_properties(pc_error PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_OUTPUT_FOLDER})
set_target_properties(pc_error PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_OUTPUT_FOLDER})
set_target_properties(pc_error PROPERTIES DEBUG_POSTFIX "_d")
