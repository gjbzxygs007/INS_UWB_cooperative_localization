//
// Created by ubuntu-jianan
//

#ifndef COOPERATIVE_CONFIG_H
#define COOPERATIVE_CONFIG_H

#include "common_include.h"

namespace cl {

class Config {
public:
    ~Config() = default;

    static void SetParameterFile(const string & filename);

    template<typename T>
    static T get(const string& key) {return T(Config::config_->file_[key]); }

private:
    Config() {}
    Config(const Config & obj) = delete;
    Config & operator=(const Config & obj) = delete;

    cv::FileStorage file_;
    static std::shared_ptr<Config> config_;
};

}



#endif //COOPERATIVE_CONFIG_H
