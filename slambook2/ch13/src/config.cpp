#include "myslam/config.h"

namespace myslam {
bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config(filename));
    // config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release();
        return false;
    }
    // std::string n = config_->Get<std::string>(std::string("dataset_dir"));
    return true;
}

Config::Config(std::string filename):config_path(filename){
    this->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
}

Config::~Config() {
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}
