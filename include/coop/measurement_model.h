#ifndef COOPERATIVE_MEASUREMENT_MODEL_H
#define COOPERATIVE_MEASUREMENT_MODEL_H

/* Define the state, jacobian of different
 *
 *
 *
 */

#include <iostream>
#include "coop/relative_measurement.h"
#include "state.h"


namespace cl {
namespace coop {
    // 9 dim state and scale measurement
    class ImuPlusRange {
    public:
        typedef StateVector<9> StateClass;
        typedef RelMeasurement<9> MeasurementClass;
        typedef RelMeasurement<9>::Measurement Measurement;
        typedef Eigen::Matrix<double, 1, 9, Eigen::RowMajor> Jacobian;
        typedef std::shared_ptr<ImuPlusRange> Ptr;

        // Constructor and destructors
        ImuPlusRange() : _state_i(nullptr), _state_j(nullptr), _meas(nullptr) {}

        /* Input:
         *  s_i    the state of agent i
         *  s_j    the state of agent j
         *  m      relative range measurement
         */
        ImuPlusRange(StateClass & s_i, StateClass & s_j, MeasurementClass & m);
        ~ImuPlusRange() = default;

        StateClass::Ptr getStatei() {return _state_i;}
        StateClass::Ptr getStatej() {return _state_j;}
        MeasurementClass::Ptr getMeasurement() {return _meas;}
        Jacobian getJacobiani() {return _jacobian_i; }

        void updateState(StateClass::Ptr & s1, StateClass::Ptr & s2);
        void updateMeasurement(MeasurementClass::Ptr & m);
        // Update the measurement Jacobian matrix
        void updateJacobian();
        void updateResidual();

    private:
        Measurement _residual;
        Jacobian _jacobian_i;
        Jacobian _jacobian_j;
        StateClass::Ptr _state_i;
        StateClass::Ptr _state_j;
        MeasurementClass::Ptr _meas;
    };

    // 3 dimensional state(x, y, theta) and relative pose measurement
//    class OdometryPlusPose {
//    public:
//        typedef StateVector<3>::State State;
//        typedef RelMeasurement<3, 3>::Measurement Measurement;
//        typedef RelMeasurement<3, 3>::JacobianMeas JacobianMeas;
//        typedef RelMeasurement<3, 3>::MeasCov MeasCov;
//
//        OdometryPlusPose(StateVector<3> & s_i, StateVector<3> & s_j, RelMeasurement<3, 3> & m) :
//        _state_i(s_i), _state_j(s_j), _meas(m) {}
//        ~OdometryPlusPose() = default;
//
//        void updateMeasurement(const Measurement & m);
//        void updateStateI(const State & s);
//        void updateStateJ(const State & s);
//        JacobianMeas getJacobian();
//        Measurement getMeasurement();
//
//
//    private:
//        Eigen::Matrix3d getRotation(double theta);
//
//        StateVector<3> _state_i;
//        StateVector<3> _state_j;
//        RelMeasurement<3, 3> _meas;
//    };


    }
}

#endif //COOPERATIVE_MEASUREMENT_MODEL_H
