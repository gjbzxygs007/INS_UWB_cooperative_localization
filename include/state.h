// Authors: jiananz1@uci.edu

#pragma once

#include "common_include.h"

namespace cl {

// Enumerator for different state dimension
enum StateType {
    kThreeDimension,
    kNineDimension,
};

template <int D>
class StateVector {
public:
    typedef Eigen::Matrix<double, D, 1, Eigen::ColMajor> State;
    typedef Eigen::Matrix<double, D, D, Eigen::ColMajor> Covariance;
    typedef std::shared_ptr<StateVector<D>> Ptr;

    StateVector() = default;
    ~StateVector() = default;

    inline void SetState (const State & s, const Covariance & c, const StateType & t) {
        state_ = s; 
        covariance_ = c; 
        type_ = t;
    }

    State state() const {return state_; }

private:
    State state_;
    Covariance covariance_;
    StateType type_;
};

} // namespace cl

