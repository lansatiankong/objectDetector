CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME objectdetection)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})

find_package(OpenCV)

include(iCubOptions)
include(iCubHelpers)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
)
add_executable(${PROJECTNAME} ${source} ${header})


target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)
#install(TARGETS /build/binobjectdetection RUNTIME DESTINATION bin)
