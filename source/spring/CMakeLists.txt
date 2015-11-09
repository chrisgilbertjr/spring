
file(GLOB spring_source "*.c")

include_directories(${spring_SOURCE_DIR}/include/spring)


if (BUILD_STATIC)
    add_library(spring_static STATIC ${spring_source})
    set_target_properties(spring_static PROPERTIES 
        ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/libs")
endif()

if (BUILD_SHARED)
    add_library(spring_shared SHARED ${spring_source})
    set_target_properties(spring_shared PROPERTIES 
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/libs")
    target_link_libraries(spring_shared m)
ENDIF()
