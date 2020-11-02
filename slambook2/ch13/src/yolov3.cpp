#include "myslam/yolov3.h"
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace std;

namespace myslam{
    
    Yolov3::Yolov3(std::string cfg_path)
    // :device(new torch::Device(torch::kCPU))
    // ,net(cfg_path.c_str(), this->device)
    {
        torch::DeviceType device_type;
        if (torch::cuda::is_available() ) {        
            device_type = torch::kCUDA;
        } 
        else {
            device_type = torch::kCPU;
        }
        device = new torch::Device(device_type);
        net = std::shared_ptr<Darknet>(new Darknet(cfg_path.c_str(), device));
        // below code is necessary
        map<string, string> *info = net->get_net_info();
        info->operator[]("height") = std::to_string(input_image_size);
    }

    void Yolov3::loadWeight(std::string weight_path){
        std::cout << "loading weight ..." << endl;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        net->load_weights(weight_path.c_str());
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        std::cout << "weight loaded ...spend time:" << time_used.count() << endl;
        net->to(*this->device);
        torch::NoGradGuard no_grad;
        net->eval();
    }

    // void Yolov3::checkRun(std::string imgPath){
    //     std::vector<std::vector<float>> vec_result;
    //     cv::Mat img(cv::imread(imgPath, cv::IMREAD_GRAYSCALE)), origin_image;
    //     cv::cvtColor(img,origin_image,cv::COLOR_GRAY2RGB);
    //     cv::namedWindow("checkYolo", cv::WINDOW_NORMAL);
    //     if(img.empty())
    //         std::cerr << "read image is fail\n";
    //     else
    //         std::cout <<"successfully load image\n" <<"input size must: " 
    //         << input_image_size << "\n";

    //     cv::Mat resize_img, imgf;
    //     cv::resize(img, resize_img, cv::Size(input_image_size, input_image_size));
    //     resize_img.convertTo(imgf, CV_32F, 1.0/255);
    //     auto img_tensor = torch::from_blob(imgf.data, {1, input_image_size, input_image_size, 3}).to(*device);
    //     img_tensor = img_tensor.permute({0,3,1,2});
    //     // std::string log_path("/home/cia1099/project/SLAMlearning/");
    //     // std::ofstream log_file;
    //     // log_file.open(log_path+"debug_log.txt", ios::trunc);
    //     // log_file << img_tensor;
    //     net.say_hello();
    //     net.to(*this->device);
    //     torch::NoGradGuard no_grad;
    //     net.eval();
    //     // auto output = net.forward(img_tensor);
    //     // auto result = net.write_results(output, 80, 0.6, 0.4); //classnum, obj_conf, iou_conf
    //     // if(result.dim() != 1){
    //     //     float w_scale = float(origin_image.cols) / input_image_size;
    //     //     float h_scale = float(origin_image.rows) / input_image_size;

    //     //     result.select(1,1).mul_(w_scale);
    //     //     result.select(1,2).mul_(h_scale);
    //     //     result.select(1,3).mul_(w_scale);
    //     //     result.select(1,4).mul_(h_scale);
    //     //     auto result_data = result.accessor<float, 2>();
    //     //     std::cout << "\n------Frame object detection-------\n";
    //     //     for(int i=0; i!=result_data.size(0); ++i){
    //     //         auto ob = result_data[i];
    //     //         std::vector<float> obj{ob[1],ob[2],ob[3],ob[4],ob[5],ob[6],ob[7]};
    //     //         vec_result.push_back(obj);
               
    //     //         for(auto bb: vec_result)
    //     //             std::cout << bb  << " ";
    //     //         std::cout << "\n";
    //     //     }
    //     //     for (int i = 0; i < result.size(0) ; i++)
    //     //     {
    //     //     cv::rectangle(origin_image, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
    //     //     cv::Point tp(result_data[i][1], result_data[i][2]);
    //     //     float score = result_data[i][5]*result_data[i][6];
    //     //     std::stringstream scorestream;
    //     //     scorestream << std::setprecision(4) << score;
    //     //     // cv::putText(origin_image, coco[result_data[i][7]]+":"+scorestream.str(),
    //     //         // tp-cv::Point(0,5),cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(0,0,255),1,cv::LINE_AA);
    //     //     }
    //         cv::imshow("checkYolo", origin_image);
    //         cv::waitKey(1);
    //     // }
    // }

    // void Yolov3::addCurrentFrame(Frame::Ptr current_frame) {
    //     std::unique_lock<std::mutex> lck(frame_data_mutex_);
    //     current_frame_ = current_frame;
    // }

    // void Yolov3::inference(){
    //     std::vector<std::vector<float>> vec_result;
    //     if(current_frame_ == nullptr) {
    //         current_frame_->SetObjectResult(vec_result); 
    //         return;
    //     }
    //     cv::Mat img(current_frame_->left_img_), origin_image;
    //     if(img.channels() == 1)
    //         cv::cvtColor(img,origin_image,cv::COLOR_GRAY2RGB);
    //     else
    //     {
    //         cv::cvtColor(img, origin_image, cv::COLOR_BGR2RGB);
    //     } 
        
    //     cv::Mat resize_img, imgf;
    //     cv::resize(img, resize_img, cv::Size(input_image_size, input_image_size));
    //     resize_img.convertTo(imgf, CV_32F, 1.0/255);
    //     auto img_tensor = torch::from_blob(imgf.data, {1, input_image_size, input_image_size, 3}).to(*device);
    //     img_tensor = img_tensor.permute({0,3,1,2});
    //     auto output = net.forward(img_tensor);
    //     auto result = net.write_results(output, 80, 0.6, 0.4); //classnum, obj_conf, iou_conf
    //     if(result.dim() != 1){
    //         float w_scale = float(origin_image.cols) / input_image_size;
    //         float h_scale = float(origin_image.rows) / input_image_size;

    //         result.select(1,1).mul_(w_scale);
    //         result.select(1,2).mul_(h_scale);
    //         result.select(1,3).mul_(w_scale);
    //         result.select(1,4).mul_(h_scale);
    //         auto result_data = result.accessor<float, 2>();
    //         std::cout << "\n------Frame object detection-------\n";
    //         for(int i=0; i!=result_data.size(0); ++i){
    //             auto ob = result_data[i];
    //             std::vector<float> obj{ob[1],ob[2],ob[3],ob[4],ob[5],ob[6],ob[7]};
    //             vec_result.push_back(obj);
               
    //             for(auto bb: vec_result)
    //                 std::cout << bb  << " ";
    //             std::cout << "\n";
    //         }
    //     }
    //     current_frame_->SetObjectResult(vec_result);
    // }

    void Yolov3::inference(Frame::Ptr frame){
        std::vector<std::vector<float>> vec_result;
        if(frame == nullptr) {
            // current_frame_->SetObjectResult(vec_result); 
            return;
        }
        cv::Mat img(frame->left_img_), origin_image;
        if(img.channels() == 1)
            cv::cvtColor(img,origin_image,cv::COLOR_GRAY2RGB);
        else
        {
            cv::cvtColor(img, origin_image, cv::COLOR_BGR2RGB);
        } 
        
        cv::Mat resize_img, imgf;
        cv::resize(origin_image, resize_img, cv::Size(input_image_size, input_image_size));
        resize_img.convertTo(imgf, CV_32F, 1.0/255);
        auto img_tensor = torch::from_blob(imgf.data, {1, input_image_size, input_image_size, 3}).to(*device);
        img_tensor = img_tensor.permute({0,3,1,2});
        // std::string log_path("/home/cia1099/project/SLAMlearning/");
        // std::ofstream log_yolo;
        // log_yolo.open(log_path+"debug_size.txt", ios::trunc);
        // log_yolo << imgf.rows <<", " << imgf.cols << ", " << imgf.channels() << std::endl;
        // log_yolo << img_tensor;

        auto output = net->forward(img_tensor);
        auto result = net->write_results(output, 80, 0.6, 0.4); //classnum, obj_conf, iou_conf
        if(result.dim() != 1){
            float w_scale = float(origin_image.cols) / input_image_size;
            float h_scale = float(origin_image.rows) / input_image_size;

            result.select(1,1).mul_(w_scale);
            result.select(1,2).mul_(h_scale);
            result.select(1,3).mul_(w_scale);
            result.select(1,4).mul_(h_scale);
            auto result_data = result.accessor<float, 2>();
            // std::cout << "\n------Frame object detection-------\n";
            for(int i=0; i!=result_data.size(0); ++i){
                auto ob = result_data[i];
                std::vector<float> obj{ob[1],ob[2],ob[3],ob[4],ob[5],ob[6],ob[7]};
                vec_result.push_back(obj);
               
                // for(auto bb: vec_result)
                //     std::cout << bb  << " ";
                // std::cout << "\n";
            }
        }
        frame->SetObjectResult(vec_result);
    }


}