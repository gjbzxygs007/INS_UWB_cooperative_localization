//
// Created by ubuntu-jianan
//

#include "config.h"
#include <mutex>


mutex mu;
namespace cl {
    void Config::SetParameterFile(const string & filename) {
        if (config_ == NULL) {
            lock_guard<mutex> guard(mu);
            if (config_ ==NULL) {
                Config * temp = new Config();
                config_ = temp;
            }
        }
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (config_->file_.isOpened() == false) {
            cerr << "Parameter file " << filename << " does not exist." << endl;
            config_->file_.release();
            return;
        }
    }

    Config::~Config() {
        if (file_.isOpened()) {
            file_.release();
        }
        if (config_ != NULL) {
            delete config_;
        }
    }

    Config * Config::config_ = NULL;
}
