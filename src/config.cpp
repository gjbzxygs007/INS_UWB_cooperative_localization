//
// Created by ubuntu-jianan
//

#include "config.h"
#include <mutex>

mutex mu;
namespace cl {
    void Config::SetParameterFile(const string & filename) {
        if (_config == NULL) {
            lock_guard<mutex> guard(mu);
            if (_config ==NULL) {
                Config * temp = new Config();
                _config = temp;
            }
        }
        _config->_file = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (_config->_file.isOpened() == false) {
            cerr << "Parameter file " << filename << " does not exist." << endl;
            _config->_file.release();
            return;
        }
    }

    Config::~Config() {
        if (_file.isOpened()) {
            _file.release();
        }
        if (_config != NULL) {
            delete _config;
        }
    }

    Config * Config::_config = NULL;
}
