add_executable(run_kitti_stereo run_kitti_stereo.cpp)
target_link_libraries(run_kitti_stereo myslam ${THIRD_PARTY_LIBS} yolov3)

add_executable(test_yolo testYolo.cpp)
target_link_libraries(test_yolo yolov3 myslam ${THIRD_PARTY_LIBS})