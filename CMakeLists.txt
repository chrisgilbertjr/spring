cmake_minimum_required(VERSION 2.8.4)

project(spring)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE
        STRING "Building release version by default. To change please specify with -D CMAKE_BUILD_TYPE=<build-type>. options are: None Debug Release RelWithDebInfo MinSizRel."
        FORCE)
endif()

#build options
option(BUILD_STATIC "Build a static library" ON)
option(BUILD_SHARED "Build a shared library" OFF)
option(BUILD_DEMOS  "Build demos" OFF)

# compile flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -ffast-math") 
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall") 

# preprocessor flags
set(CMAKE_C_FLAGS_RELEASE "-DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-DDEBUG")

if(MSVC)
    set(CMAKE_C_FLAGS "-DSPRING_EXPORT")
endif()

add_subdirectory(source/spring)
