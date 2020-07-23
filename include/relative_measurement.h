//
// Created by ubuntu-jianan on 7/18/20.
//

#ifndef COOPERATIVE_MEASUREMENT_H
#define COOPERATIVE_MEASUREMENT_H

#include <iostream>
#include <Eigen/Core>


namespace cl {
    template <int D, int E>
    class RelMeasurement {
    public:
        static const int dim_state = D;
        static const int dim_mea = E;
        typedef Eigen::Matrix<double, E, 1, Eigen::ColMajor> Measurement;
        typedef Eigen::Matrix<double, D, 1, Eigen::ColMajor> State;
        typedef Eigen::Matrix<double, E, D, Eigen::ColMajor> Jacobian;

        RelMeasurement() {}
        virtual ~RelMeasurement() {}

        inline void SetState (const State & s) {_state = s; }
        inline void SetMeasure (const Measurement & mea) {_measurement = mea; }
        virtual Jacobian GetJacobian() = 0;
        virtual Measurement GetResidual() = 0;

    protected:
        Measurement _measurement;
        State _state;
    };

    template <int D, int E>
    class RelRangeMeasurement : public RelMeasurement<D, E> {
    public:
        typedef RelMeasurement::Measurement Measurement;
        typedef RelMeasurement::State State;
        typedef RelMeasurement::Jacobian Jacobian;

        virtual Jacobian GetJacobian() {




        }



    };



}


#endif //COOPERATIVE_MEASUREMENT_H
