set(APPLICATION_INCLUDE_DIRS 
    include
    /usr/arm-linux-gnueabi/include
    ${CMAKE_SOURCE_DIR}/PHOENIX/include
    ${CMSISDSP}/Include
    ${CMSISDSP}/PrivateInclude
    ${CMSISDSP}/Include/dsp
)
include_directories(
    ${APPLICATION_INCLUDE_DIRS}
)

file(GLOB_RECURSE APPLICATION_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(application ${APPLICATION_SOURCES})

target_link_libraries(application
    phoenix
)