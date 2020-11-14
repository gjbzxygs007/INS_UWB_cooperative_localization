// Authors: jiananz1@uci.edu

#include "coop/measurement_model.h"

#include "config.h"

namespace cl {
namespace coop {

ImuPlusRange::ImuPlusRange() {
    measurement_type_ = MeasurementType::kRange;
    state_type_ = StateType::kNineDimension;
    jacobian_i_ = Jacobian::Zero();
    jacobian_j_ = Jacobian::Zero();
    residual_ = Measurement::Zero();
    measurement_covariance_ << Config::get<double>("variance_of_range");
    Measurement meas = Measurement::Zero();
    measurement_ptr_ = std::make_shared<MeasurementClass>(meas, measurement_type_);
    state_i_ = State::Zero();
    state_j_ = State::Zero();
}

// \brief update the current relative measurement
void ImuPlusRange::UpdateMeasurement(const Measurement & meas) {
    measurement_ptr_->SetMeasurement(meas);
}

// \brief update the current states of i and j
void ImuPlusRange::UpdateState(const State & s1, const State & s2) {
    state_i_ = s1;
    state_j_ = s2;
}

void ImuPlusRange::UpdateResidual() {
    Measurement measurement_estimated;
    measurement_estimated << RangingModel();
    residual_ = measurement_ptr_->measurement() - measurement_estimated;
}

// \brief return measurement jacobian matrix
void ImuPlusRange::UpdateJacobian() {
    double x1 = state_i_(0, 0);
    double y1 = state_i_(1, 0);
    double z1 = state_i_(2, 0);
    double x2 = state_j_(0, 0);
    double y2 = state_j_(1, 0);
    double z2 = state_j_(2, 0);
    double r = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
    jacobian_i_(0, 0) = (x1 - x2) / r;
    jacobian_i_(1, 0) = (y1 - y2) / r;
    jacobian_i_(2, 0) = (z1 - z2) / r;
    jacobian_j_(0, 0) = (x2 - x1) / r;
    jacobian_j_(1, 0) = (y2 - y1) / r;
    jacobian_j_(2, 0) = (z2 - z1) / r;
}


// \brief: the UWB ranging model h(xi, xj)
double ImuPlusRange::RangingModel() {
    double sd1 = state_i_(0, 0) - state_j_(0, 0);
    double sd2 = state_i_(1, 0) - state_j_(1, 0);
    double sd3 = state_i_(2, 0) - state_j_(2, 0);
    return sqrt(sd1 * sd1 + sd2 * sd2 + sd3 * sd3);
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


} // namespace coop
} // namespace cl

