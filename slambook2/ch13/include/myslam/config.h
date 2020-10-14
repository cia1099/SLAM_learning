#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * 配置类，使用SetParameterFile确定配置文件
 * 然后用Get得到对应值
 * 单例模式
 */
class Config {
   private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
    std::string config_path;

    Config() {}  // private constructor makes a singleton
    Config(std::string);
   public:
//    cv::FileStorage file_;
    ~Config();  // close the file when deconstructing

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string &key) {
        cv::FileStorage fs(Config::config_->config_path, cv::FileStorage::READ); //= Config::config_->file_;
        // if(fs.isOpened())
        //     std::cout << "cv::FileStorage is opening\n";
        // cv::FileNode n = fs[key];
        // T name = (T)(*n.begin());
        // return name;
        // cv::FileStorage fs = Config::config_->file_;
        T name = T(fs[key]);
        if(fs.isOpened())
            fs.release();
        return name;
        // return T(Config::config_->file_[key]);
    }
};
}  // namespace myslam

#endif  // MYSLAM_CONFIG_H
