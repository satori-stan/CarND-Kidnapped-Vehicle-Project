project(PARTICLE_FILTER)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/particle_filter.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

include_directories("C:/Program\ Files\ (x86)/vcpkg/installed/x86-windows/include")  # At least if using vcpkg
link_directories("C:/Program\ Files\ (x86)/vcpkg/installed/x86-windows/include/openssl")  # At least if using vcpkg
link_directories("C:/Program\ Files\ (x86)/vcpkg/installed/x86-windows/lib")  # At least if using vcpkg

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

add_executable(particle_filter ${sources})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
  # At least if using vcpkg
  target_link_libraries(particle_filter zlib ssleay32 libuv uWS)

else(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 
  target_link_libraries(particle_filter z ssl uv uWS)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows") 

