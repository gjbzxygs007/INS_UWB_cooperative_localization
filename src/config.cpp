//
// Created by ubuntu-jianan
//

#include "config.h"
#include <mutex>

mutex mu;
namespace cl {

std::shared_ptr<Config> Config::config_ = nullptr;

void Config::SetParameterFile(const string & filename) {
    if (config_ == nullptr) {
        lock_guard<mutex> guard(mu);
        if (config_ == nullptr) {
            config_ = std::shared_ptr<Config>(new Config);
        }
    }

    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        cerr << "Parameter file " << filename << " does not exist." << endl;
        config_->file_.release();
        return;
    }
}

} // namespace cl
