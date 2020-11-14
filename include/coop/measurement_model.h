// Authors: jiananz1@uci.edu

#pragma once

#include <iostream>
#include "coop/relative_measurement.h"
#include "state.h"


namespace cl {
namespace coop {

// 9 dim state and scalar measurement
class ImuPlusRange final {
public:
    typedef RelativeMeasurement<1> MeasurementClass;
    typedef StateVector<9>::State State;
    typedef MeasurementClass::Measurement Measurement;
    typedef Eigen::Matrix<double, 1, 9, Eigen::RowMajor> Jacobian;
    typedef Eigen::Matrix<double, 1, 1, Eigen::RowMajor> MeasurementCovariance;

    // Constructor and destructors
    ImuPlusRange();
    ~ImuPlusRange() = default;

    // Accessors
    MeasurementClass::Ptr measurement_ptr() {return measurement_ptr_;}
    Measurement residual() {return residual_;}
    MeasurementCovariance measurement_covariance() {return measurement_covariance_;}
    Jacobian jacobian_i() {return jacobian_i_;}
    Jacobian jacobian_j() {return jacobian_j_;}
    State state_i() {return state_i_;}
    State state_j() {return state_j_;}

    // update the relative measurement
    void UpdateMeasurement(const Measurement& meas);

    // update the local state of each agent
    void UpdateState(const State& s1, const State& s2);

    // Update the residual once a new relative measurement is detected
    void UpdateResidual();

    // Update the measurement Jacobian matrix
    void UpdateJacobian();

private:
    // The relative ranging model
    double RangingModel();

    MeasurementType measurement_type_;
    StateType state_type_;
    MeasurementClass::Ptr measurement_ptr_;
    Measurement residual_;
    MeasurementCovariance measurement_covariance_;
    Jacobian jacobian_i_;
    Jacobian jacobian_j_;
    State state_i_;
    State state_j_;

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

} // namespace coop
} // namespace cl

