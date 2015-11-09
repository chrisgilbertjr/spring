
file(GLOB spring_source "*.c")

include_directories(${spring_SOURCE_DIR}/include/spring)

add_library(spring_static STATIC ${spring_source})
