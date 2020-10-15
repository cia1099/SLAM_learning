project(darknet)
include_directories(${DarkNet_DIR})
add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

add_library(yolov3 SHARED ${DarkNet_DIR}Darknet.cpp)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS} yolov3)

