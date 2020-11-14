// Authors: jiananz1@uci.edu

#pragma once

#include "common_include.h"

namespace cl {

class Config {
public:
    ~Config() = default;

    static void SetParameterFile(const std::string & filename);

    template<typename T>
    static T get(const std::string& key) {return T(Config::config_->file_[key]); }

private:
    Config() = default;
    Config(const Config & obj) = delete;
    Config & operator=(const Config & obj) = delete;

    cv::FileStorage file_;
    static std::shared_ptr<Config> config_;
};

} // namespace cl

