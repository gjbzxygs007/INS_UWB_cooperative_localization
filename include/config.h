//
// Created by ubuntu-jianan
//

#ifndef COOPERATIVE_CONFIG_H
#define COOPERATIVE_CONFIG_H

#include "common_include.h"

namespace cl {

class Config {
public:
    ~Config();

    static void SetParameterFile(const string & filename);

    template<typename T>
    static T get(const string & key) {return T(Config::_config->_file[key]); }

private:
    Config() = default;
    Config(const Config & obj) = delete;
    Config & operator=(const Config & obj) = delete;

    cv::FileStorage file_;
    static Config * config_;
};

}



#endif //COOPERATIVE_CONFIG_H
