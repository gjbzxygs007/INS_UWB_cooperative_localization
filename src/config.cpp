// Authors: jiananz1@uci.edu

#include "common_include.h"
#include "config.h"

namespace {

mutex mu;

} // namespace

namespace cl {

std::shared_ptr<Config> Config::config_ = nullptr;

void Config::SetParameterFile(const string & filename) {
    if (config_ == nullptr) {
        lock_guard<mutex> guard(mu);
        if (config_ == nullptr) {
            config_ = std::shared_ptr<Config>(new Config);
        }
    }

    config_->file_.open(filename, cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        cerr << "Parameter file " << filename << " does not exist." << endl;
        config_->file_.release();
        return;
    }
}

} // namespace cl
