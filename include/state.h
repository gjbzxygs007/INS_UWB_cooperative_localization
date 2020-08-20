#ifndef COOPERATIVE_STATE_H
#define COOPERATIVE_STATE_H

#include "common_include.h"

namespace cl {
    enum StateType {
        threedim,
        ninedim,
    };

    template <int D>
    class StateVector {
    public:
        static const int dim_state = D;
        typedef Eigen::Matrix<double, D, 1, Eigen::ColMajor> State;
        typedef std::shared_ptr<StateVector<D>> Ptr;

        StateVector() = default;
        ~StateVector() = default;

        inline void setState (const State & s, StateType & t) {_state = s; _type = t;}
        inline State getState() {return _state; }

    private:
        State _state;
        StateType _type;
    };
}

#endif //COOPERATIVE_STATE_H
