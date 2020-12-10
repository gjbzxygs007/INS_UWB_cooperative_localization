// Authors: jiananz1@uci.edu

#include "coop/cooperative.h"

#include <stdlib.h>

#include "config.h"

namespace cl {
namespace coop {

CooperativeImu::CooperativeImu() {
    measurement_mode_ptr_ = std::make_shared<ImuPlusRange>();
    connector_ptr_ = std::make_shared<Connector>();
    std::string path = Config::get<std::string>("uwb_path");
    connector_ptr_->Initialize(path.c_str());

    // Read the MAC addresses of agents from the yaml file
    auto file_node_agent = Config::get<cv::FileNode>("addresses_of_agents");
    if (file_node_agent.type() != cv::FileNode::SEQ) {
        std::cerr << "strings is not a sequence! FAIL" << std::endl;
        exit(1);
    }
    cv::FileNodeIterator it_agent = file_node_agent.begin();
    cv::FileNodeIterator it_end_agent = file_node_agent.end();
    int id = 0;
    for (; it_agent != it_end_agent; ++it_agent) {
        address_to_id_agent_[std::string(*it_agent)] = id;
        id++;
    }

    // Read the MAC addresses of beacons from the yaml file
    auto file_node_beacon = Config::get<cv::FileNode>("addresses_of_beacons");
    if (file_node_beacon.type() != cv::FileNode::SEQ) {
        std::cerr << "strings is not a sequence! FAIL" << std::endl;
        exit(1);
    }
    cv::FileNodeIterator it_beacon = file_node_beacon.begin();
    cv::FileNodeIterator it_end_beacon = file_node_beacon.end();
    id = 0;
    for (; it_beacon != it_end_beacon; ++it_beacon) {
        address_to_id_beacon_[std::string(*it_beacon)] = id;
        id++;
    }

    
}



} //namespace coop
} //namespace cl