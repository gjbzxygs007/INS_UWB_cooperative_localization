#ifndef COOPERATIVE_STATE_H
#define COOPERATIVE_STATE_H

#include "common_include.h"

namespace cl {
    enum StateType {
        THREE_DIM,
        NINE_DIM,
    };

    template <int D>
    class StateVector {
    public:
        static const int dim_state = D;
        typedef Eigen::Matrix<double, D, 1, Eigen::ColMajor> State;
        typedef Eigen::Matrix<double, D, D, Eigen::ColMajor> Covariance;
        typedef std::shared_ptr<StateVector<D>> Ptr;

        StateVector() = default;
        ~StateVector() = default;

        inline void setState (const State & s, const Covariance & c, const StateType & t) {_state = s; _cov = c; _type = t;}
        inline State getState() {return _state; }

    private:
        State _state;
        Covariance _cov;
        StateType _type;
    };
}

#endif //COOPERATIVE_STATE_H
