//
// Created by ubuntu-jianan
//

#include <cmath>
#include "coop/measurement_model.h"
#include "config.h"

namespace cl {
namespace coop {
    ImuPlusRange::ImuPlusRange() {
        _type = RANGE;
        _type_s = THREE_DIM;
        _jacobian_i = Jacobian::Zero();
        _jacobian_j = Jacobian::Zero();
        _residual = Measurement::Zero();
        double r = Config::get<double>("variance_of_range");
        _cov_meas << r;
        Measurement m = Measurement::Zero();
        _meas = make_shared<MeasurementClass>(m, _type);
        _state_i = State::Zero();
        _state_j = State::Zero();
    }

    // \brief update the current relative measurement
    void ImuPlusRange::updateMeasurement(Measurement & m) {
        _meas->setMeasurement(m);
    }

    // \brief update the current states of i and j
    void ImuPlusRange::updateState(State & s1, State & s2) {
        _state_i = s1;
        _state_j = s2;
    }

    // \brief: the UWB ranging model h(xi, xj)
    double ImuPlusRange::rangingModel() {
        double sd1 = _state_i(0, 0) - _state_j(0, 0);
        double sd2 = _state_i(1, 0) - _state_j(1, 0);
        double sd3 = _state_i(2, 0) - _state_j(2, 0);
        return sqrt(sd1 * sd1 + sd2 * sd2 + sd3 * sd3);
    }

    void ImuPlusRange::updateResidual() {
        Measurement meas_estimated;
        meas_estimated << rangingModel();
        _residual = _meas->getMeasurement() - meas_estimated;
    }

    // \brief return measurement jacobian matrix
    void ImuPlusRange::updateJacobian() {
        double x1 = _state_i(0, 0);
        double y1 = _state_i(1, 0);
        double z1 = _state_i(2, 0);
        double x2 = _state_j(0, 0);
        double y2 = _state_j(1, 0);
        double z2 = _state_j(2, 0);
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

