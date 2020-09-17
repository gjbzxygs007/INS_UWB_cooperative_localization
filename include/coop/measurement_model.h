#ifndef COOPERATIVE_MEASUREMENT_MODEL_H
#define COOPERATIVE_MEASUREMENT_MODEL_H

/* Define the state, jacobian of different
 * information needed for CL update
 */

#include <iostream>
#include "coop/relative_measurement.h"
#include "state.h"


namespace cl {
namespace coop {
    // 9 dim state and scale measurement
    class ImuPlusRange {
    public:
        typedef RelMeasurement<1> MeasurementClass;
        typedef StateVector<9> StateClass;
        typedef MeasurementClass::Measurement Measurement;
        typedef Eigen::Matrix<double, 1, 9, Eigen::RowMajor> Jacobian;

        // Constructor and destructors
        ImuPlusRange() : _meas(nullptr) {}

        /* Input:
         *  s_i    the state of agent i
         *  s_j    the state of agent j
         *  m      relative range measurement
         */
        ImuPlusRange(Ptr & m);
        virtual ~ImuPlusRange() {}
//
    // Accessors
        MeasurementClass::Ptr getMeasurement() {return _meas;}
        Measurement getResidual() {return _residual;}
        Jacobian getJacobiani() {return _jacobian_i;}
        Jacobian getJacobianj() {return _jacobian_j;}
        StateClass::Ptr getStatei() {return _state_i;}
        StateClass::Ptr getStatej() {return _state_j;}

        // update the relative measurement
        void updateMeasurement(Measurement & m);

        // update the local state of each agent
        void updateState(StateClass::Ptr & s1, StateClass::Ptr & s2);

        // Update the residual once a new relative measurement is detected
        void updateResidual();

        // Update the measurement Jacobian matrix
        void updateJacobian(StateClass::Ptr & s1, StateClass::Ptr & s2);
    private:
        MeasurementClass::Ptr _meas;
        Measurement _residual;
        Jacobian _jacobian_i;
        Jacobian _jacobian_j;
        StateClass::Ptr _state_i;
        StateClass::Ptr _state_j;
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
