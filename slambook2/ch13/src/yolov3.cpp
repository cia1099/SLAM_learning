#include "myslam/yolov3.h"
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>

using namespace std;

namespace myslam{
    
    Yolov3::Yolov3(std::string cfg_path):
    device(new torch::Device(torch::kCPU)),
    net(cfg_path.c_str(), this->device)
    {}

    void Yolov3::loadWeight(std::string weight_path){
        std::cout << "loading weight ..." << endl;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        net.load_weights(weight_path.c_str());
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        std::cout << "weight loaded ...spend time:" << time_used.count() << endl;
    }

    void Yolov3::addCurrentFrame(Frame::Ptr current_frame) {
        std::unique_lock<std::mutex> lck(frame_data_mutex_);
        current_frame_ = current_frame;
    }

    void Yolov3::inference(){
        if(current_frame_ == nullptr) return;
        cv::Mat img(current_frame_->left_img_), origin_image;
        if(img.channels() == 1)
            cv::cvtColor(img,origin_image,cv::COLOR_GRAY2RGB);
        else
        {
            cv::cvtColor(img, origin_image, cv::COLOR_BGR2RGB);
        }
        std::vector<std::vector<float>> vec_result; 
        
        cv::Mat resize_img, imgf;
        cv::resize(img, resize_img, cv::Size(input_image_size, input_image_size));
        resize_img.convertTo(imgf, CV_32F, 1.0/255);
        auto img_tensor = torch::from_blob(imgf.data, {1, input_image_size, input_image_size, 3}).to(*device);
        img_tensor = img_tensor.permute({0,3,1,2});
        auto output = net.forward(img_tensor);
        auto result = net.write_results(output, 80, 0.6, 0.4); //classnum, obj_conf, iou_conf
        if(result.dim() != 1){
            float w_scale = float(origin_image.cols) / input_image_size;
            float h_scale = float(origin_image.rows) / input_image_size;

            result.select(1,1).mul_(w_scale);
            result.select(1,2).mul_(h_scale);
            result.select(1,3).mul_(w_scale);
            result.select(1,4).mul_(h_scale);
            auto result_data = result.accessor<float, 2>();
            for(int i=0; i!=result_data.size(0); ++i){
                auto ob = result_data[i];
                std::vector<float> obj{ob[1],ob[2],ob[3],ob[4],ob[5],ob[6],ob[7]};
                vec_result.push_back(obj);
            }
        }
        current_frame_->SetObjectResult(vec_result);
    }

}