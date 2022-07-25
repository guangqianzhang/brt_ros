

find_package(OpenCV 4.5.5 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBRARIES})