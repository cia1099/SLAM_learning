#pragma once

#ifndef MYSLAM_OBJECTDETECTION_H
#define MYSLAM_OBJECTDETECTION_H

#include "myslam/frame.h"
// #include "Darknet.h"
#include "/home/cia1099/project/ML/libtorch-yolov3/Darknet.h"

namespace myslam{
    class Yolov3{
    public:
    typedef std::shared_ptr<Yolov3> Ptr;

        Yolov3(std::string);
        void loadWeight(std::string);
        // void addCurrentFrame(Frame::Ptr);
        // void inference();
        void inference(Frame::Ptr);
        // void checkRun(std::string);
    private: 
        std::shared_ptr<Darknet> net;
        torch::Device* device;
        Frame::Ptr current_frame_ = nullptr;
        std::mutex frame_data_mutex_;
        const int input_image_size = 416;
    };
}

#endif //MYSLAM_OBJECTDETECTION_H