//
// Created by ubuntu-jianan
//

#include <cmath>
#include "coop/measurement_model.h"

namespace cl {
namespace coop {
    ImuPlusRange::ImuPlusRange(StateClass & s_i, StateClass & s_j, MeasurementClass & m) {
        _state_i = std::make_shared<StateClass> (s_i);
        _state_j = std::make_shared<StateClass> (s_j);
        _meas = std::make_shared<MeasurementClass> (m);
        _jacobian_i = Jacobian::Zero();
        _jacobian_j = Jacobian::Zero();
        _residual = Measurement::Zero();
    }

    // \brief update the current states of i and j
    void ImuPlusRange::updateState(StateClass::Ptr & s1, StateClass::Ptr & s2) {
        _state_i = s1;
        _state_j = s2;
    }

    // \brief update the current relative measurement
    void ImuPlusRange::updateMeasurement(MeasurementClass::Ptr & m) {
        _meas = m;
    }

    // \brief update the estimated relative measurement
    void ImuPlusRange::updateResidual() {
        Measurement meas;
        double x1 = _state_i->getState()(0, 0);
        double y1 = _state_i->getState()(1, 0);
        double z1 = _state_i->getState()(2, 0);
        double x2 = _state_j->getState()(0, 0);
        double y2 = _state_j->getState()(1, 0);
        double z2 = _state_j->getState()(2, 0);
        meas(0, 0) = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
        _residual = _meas->getMeasurement() - meas;
    }

    // \brief return measurement jacobian matrix
    void ImuPlusRange::updateJacobian() {
        double x1 = _state_i->getState()(0, 0);
        double y1 = _state_i->getState()(1, 0);
        double z1 = _state_i->getState()(2, 0);
        double x2 = _state_j->getState()(0, 0);
        double y2 = _state_j->getState()(1, 0);
        double z2 = _state_j->getState()(2, 0);
        double r = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
        _jacobian_i(0, 0) = (x1 - x2) / r;
        _jacobian_i(1, 0) = (y1 - y2) / r;
        _jacobian_i(2, 0) = (z1 - z2) / r;
        _jacobian_j(0, 0) = (x2 - x1) / r;
        _jacobian_j(1, 0) = (y2 - y1) / r;
        _jacobian_j(2, 0) = (z2 - z1) / r;
    }




//
//    void OdometryPlusPose::updateMeasurement(const Measurement & m) {
//        _meas.setMeasurement(m);
//    }
//
//    void OdometryPlusPose::updateStateI(const State & s) {
//        _state_i.setState(s);
//    }
//
//    void OdometryPlusPose::updateStateJ(const State & s) {
//        _state_j.setState(s);
//    }
//
//    OdometryPlusPose::JacobianMeas OdometryPlusPose::getJacobian() {
//        Eigen::Matrix3d matrix_rel;
//        matrix_rel << 1, 0, -(_state_j.getState()(1, 0) - _state_i.getState()(1, 0)),
//        0, 0, _state_j.getState()(0, 0) - _state_i.getState()(0, 0),
//        0, 0, 1;
//        return getRotation(_state_i.getState()(2, 0)) * matrix_rel;
//    }
//
//
//        OdometryPlusPose::Measurement OdometryPlusPose::getMeasurement() {
//            return getRotation(_state_i.getState()(2, 0)) * (_state_j.getState() - _state_i.getState());
//        }
//
//        Eigen::Matrix3d OdometryPlusPose::getRotation(double theta) {
//            Eigen::Matrix3d r;
//            r << cos(theta), -sin(theta), 0,
//                 sin(theta), cos(theta), 0,
//                 0, 0, 1;
//            return r;
//        }


    }
}

