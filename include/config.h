//
// Created by ubuntu-jianan
//

#ifndef COOPERATIVE_CONFIG_H
#define COOPERATIVE_CONFIG_H

namespace cl {
    class Config {
    private:
        cv::FileStorage file_;
        static Config * config_;

        Config()=default;
        Config(const Config & obj)=delete;
        Config & operator=(const Config & obj)=delete;

    public:
        ~Config();

        static void SetParameterFile(const string & filename);
        template<typename T>
        static T get(const string & key) {return T(Config::config_->file_[key]); }
    };

}



#endif //COOPERATIVE_CONFIG_H
