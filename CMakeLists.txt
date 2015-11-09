cmake_minimum_required(VERSION 2.8.4)

project(spring)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# always use gnu99
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")


# debug/release flags
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffast-math") 
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall") 

# preprocessor
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-DDEBUG")

if(MSVC)
    set(CMAKE_C_FLAGS "-DSPRING_EXPORT")
endif()

add_subdirectory(source/spring)
