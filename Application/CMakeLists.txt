set(APPLICATION_INCLUDE_DIRS 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PHOENIX_INCLUDE_DIRS}
    CACHE INTERNAL "APPLICATION include directories"
)
include_directories(
    ${APPLICATION_INCLUDE_DIRS}
)

file(GLOB_RECURSE APPLICATION_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(application ${APPLICATION_SOURCES})

target_link_libraries(application
    phoenix
)