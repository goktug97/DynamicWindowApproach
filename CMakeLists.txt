cmake_minimum_required(VERSION 2.8)
project(DynamicWindowApproach)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(DWA_VERSION_MAJOR 1)
set(DWA_VERSION_MINOR 0)
set(DWA_VERSION_PATCH 3)
set(DWA_VERSION_STRING ${DWA_VERSION_MAJOR}.${DWA_VERSION_MINOR}.${DWA_VERSION_PATCH})

# Build demo executable
include_directories(src)
add_executable(demo src/demo.c src/dwa.c)
set_target_properties(demo
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "../bin")
target_link_libraries(demo m)

# Build shared library
add_library(dwa SHARED src/dwa.c)
target_link_libraries(dwa m)
set_target_properties(dwa PROPERTIES PUBLIC_HEADER "src/dwa.h"
  VERSION ${DWA_VERSION_STRING}
  SOVERSION ${DWA_VERSION_MAJOR})
INSTALL(TARGETS dwa 
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include)
