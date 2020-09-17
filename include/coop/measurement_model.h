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
        typedef StateVector<9>::State State;
        typedef MeasurementClass::Measurement Measurement;
        typedef Eigen::Matrix<double, 1, 9, Eigen::RowMajor> Jacobian;
        typedef Eigen::Matrix<double, 1, 1, Eigen::RowMajor> MeasCov;

        // Constructor and destructors
        ImuPlusRange();
        ~ImuPlusRange() {}
//
    // Accessors
        MeasurementClass::Ptr getMeasurement() {return _meas;}
        Measurement getResidual() {return _residual;}
        MeasCov getMeasurementCov() {return _cov_meas;}
        Jacobian getJacobiani() {return _jacobian_i;}
        Jacobian getJacobianj() {return _jacobian_j;}
        State getStatei() {return _state_i;}
        State getStatej() {return _state_j;}

        // update the relative measurement
        void updateMeasurement(Measurement & m);

        // update the local state of each agent
        void updateState(State & s1, State & s2);

        // Update the residual once a new relative measurement is detected
        void updateResidual();

        // Update the measurement Jacobian matrix
        void updateJacobian();

    private:
        MeasType _type;
        StateType _type_s;
        MeasurementClass::Ptr _meas;
        Measurement _residual;
        MeasCov _cov_meas;
        Jacobian _jacobian_i;
        Jacobian _jacobian_j;
        State _state_i;
        State _state_j;

        // The relative ranging model
        double rangingModel();
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
