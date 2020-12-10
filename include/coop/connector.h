// Authors: jiananz1@uci.edu

#pragma once

#include "common_include.h"
#include "config.h"

namespace cl {
namespace coop {

struct Message {
    std::string id;
    std::vector<double> state;
    std::vector<std::vector<double>> covariance;
    double range;
    double power;
    Message(const std::string& i,
        const std::vector<double>& s,
        const std::vector<std::vector<double>>& c, double r, double p) :
        id(i), state(s), covariance(c), range(r), power(p) {}
};

class Connector final {
public:
    typedef std::shared_ptr<Connector> Ptr;

    Connector() {
        const char * serial_port = Config::get<std::string>("uwb_path").c_str();
        Initialize(serial_port);
    }

    ~Connector() = default;

    void Initialize(const char * serial_port);
    bool Write(const Message& msg);
    bool Read(Message* msg);
    bool Parse(const char* buf, int num, Message* msg);

private:
    int fd_ = -1;

};

} // namespace coop
} // namespace cl
