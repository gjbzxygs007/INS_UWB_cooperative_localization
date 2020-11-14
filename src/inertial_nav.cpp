// Authors: jiananz1@uci.edu

#include "inertial/inertial_nav.h"

#include <math.h>

#include "config.h"

namespace inertial {

Inertial::Inertial(double roll, double pitch) {
    double sigma_acc = cl::Config::get<double>("variance_of_acceleration");
    double sigma_gyro = cl::Config::get<double>("variance_of_gyro");
    double variance_position = cl::Config::get<double>("varaince_of_initial_position");
    double variance_speed = cl::Config::get<double>("variance_of_initial_speed");
    double variance_attitude = cl::Config::get<double>("variance_of_initial_attitude");
    window_size_ = cl::Config::get<int>("zero_velocity_detector_window_size");
	sigma_ = cl::Config::get<double>("zero_velocity_detector_sigma");
	double variance_zupt = cl::Config::get<double>("zero_velocity_update_variance");
	covariance_zupt_ << pow(variance_zupt, 2), 0, 0,
		 			    0, pow(variance_zupt, 2), 0,
		                0, 0, pow(variance_zupt, 2);
	variance_position *= variance_position;
    variance_speed *= variance_speed;
    variance_attitude *= variance_attitude;

    state_ = Vector9d::Zero();
    state_(6) = roll;
    state_(7) = pitch;
    state_(8) = cl::Config::get<double>("heading_angle");

    covariance_ << pow(variance_position, 2), 0, 0, 0, 0, 0, 0, 0, 0,
					0, pow(variance_position, 2), 0, 0, 0, 0, 0, 0, 0,
				    0, 0, pow(variance_position, 2), 0, 0, 0, 0, 0, 0,
		 		    0, 0, 0, pow(variance_speed, 2), 0, 0, 0, 0, 0,
				    0, 0, 0, 0, pow(variance_speed, 2), 0, 0, 0, 0,
				    0, 0, 0, 0, 0, pow(variance_speed, 2), 0, 0, 0,
					0, 0, 0, 0, 0, 0, pow(variance_attitude, 2), 0, 0,
					0, 0, 0, 0, 0, 0, 0, pow(variance_attitude, 2), 0,
					0, 0, 0, 0, 0, 0, 0, 0, pow(variance_attitude, 2);

    covariance_imu_ << pow(sigma_acc,2), 0, 0, 0, 0, 0,
		 			   0, pow(sigma_acc,2), 0, 0, 0, 0,
		 			   0, 0, pow(sigma_acc,2), 0, 0, 0,
		               0, 0, 0, pow(sigma_gyro,2), 0, 0,
	                   0, 0, 0, 0, pow(sigma_gyro,2), 0,
                       0, 0, 0, 0, 0, pow(sigma_gyro,2);
    ConvertEulerToDcm();
	ConvertDcmToQuaternion();
}

void Inertial::Propagate(const Vector3d& readouts_acc, const Vector3d& readouts_gyro, double dt) {
	UpdateQuaternion(readouts_gyro, dt);
	ConvertQuaternionToDcm();
	state_(6) = atan2(dcm_matrix_(2, 1), dcm_matrix_(2, 2));
	state_(7) = -atan(dcm_matrix_(2, 0) / sqrt(1 - pow(dcm_matrix_(2, 0), 2)));
	state_(8) = atan2(dcm_matrix_(1, 0), dcm_matrix_(0, 0));
	// Rotate the accelerations from local to navigation frame;	
	Vector3d readouts_acc_global = dcm_matrix_ * readouts_acc;
 	Vector3d gravity_vector;
 	gravity_vector << 0, 0, kGravity;
	Vector3d accelerations = readouts_acc_global + gravity_vector;
	Vector3d position, velocity;
	position = state_.segment(0, 3) + state_.segment(3, 3) * dt / 1000 + accelerations * 0.5 * pow(dt / 1000, 2);
	velocity = state_.segment(3, 3) + accelerations * dt / 1000;
	state_.segment(0, 3) = position;
	state_.segment(3, 3) = velocity;
 	Matrix3d s_t;
    s_t << 0, -readouts_acc_global(2), readouts_acc_global(1),
    	   readouts_acc_global(2), 0, -readouts_acc_global(0),
    	   -readouts_acc_global(1), readouts_acc_global(0), 0;
   	MatrixNbN state_transition;
    MatrixNbS input_transition;
    state_transition << Matrix3d::Identity(), Matrix3d::Identity() * dt / 1000,Matrix3d::Zero(),
    	   				Matrix3d::Zero(), Matrix3d::Identity(),s_t * dt / 1000,
    	   				Matrix3d::Zero(), Matrix3d::Zero(), Matrix3d::Identity();
    input_transition << Matrix3d::Zero(), Matrix3d::Zero(),
    	   				dcm_matrix_ * dt / 1000, Matrix3d::Zero(),
    	   				Matrix3d::Zero(), -dcm_matrix_ * dt / 1000;
    MatrixNbN temp;
    temp = state_transition * covariance_ * state_transition.transpose() + input_transition * covariance_imu_ * input_transition.transpose();
    covariance_ = (temp + temp.transpose()) * 0.5;
}

bool Inertial::IsZeroVelocity(const Vector3d & readouts_gyro) {
	double sigma_2g = pow(sigma_ * kPi / 180, 2);
	double indication = pow(readouts_gyro.norm(), 2) / (sigma_2g * window_size_);
	return indication < threshold_;
}

void Inertial::UpdateZeroVelocity() {
	MatrixNbT kalman_gain;
 	Vector3d residual;
 	Vector9d delta_state;
 	Matrix3d covariance_residual;
 	MatrixTbN jacobian_zupt;
 	jacobian_zupt << Matrix3d::Zero(), Matrix3d::Identity(), Matrix3d::Zero();

 	covariance_residual = jacobian_zupt * covariance_ * jacobian_zupt.transpose() + covariance_zupt_;
 	kalman_gain = covariance_ * jacobian_zupt.transpose() * covariance_residual.inverse();
 	residual = -state_.segment(3, 3);
	delta_state = kalman_gain * residual;
	state_ += delta_state;

	Vector3d epsilon = delta_state.segment(6, 3);
	Matrix3d omega;
	omega << 0, -epsilon(2), epsilon(1),
		     epsilon(2), 0, -epsilon(0),
	         -epsilon(1), epsilon(0), 0;
	dcm_matrix_ = (Matrix3d::Identity() - omega) * dcm_matrix_;
    state_(6) = atan2(dcm_matrix_(2, 1), dcm_matrix_(2, 2));
	state_(7) = -atan(dcm_matrix_(2, 0) / sqrt(1 - pow(dcm_matrix_(2, 0), 2)));
	state_(8) = atan2(dcm_matrix_(1, 0), dcm_matrix_(0, 0));
	ConvertDcmToQuaternion();
	covariance_ = covariance_ - kalman_gain * jacobian_zupt * covariance_;
	covariance_ = (covariance_ + covariance_.transpose()) / 2;
}

void Inertial::UpdateQuaternion(const Vector3d & readouts_gyro, double dt) {
	double p = readouts_gyro(0) * dt / 1000;
	double q = readouts_gyro(1) * dt / 1000;
	double r = readouts_gyro(2) * dt / 1000;
	double sigma = dt / 1000 * sqrt(pow(readouts_gyro(0) ,2) + pow(readouts_gyro(1), 2) + pow(readouts_gyro(2), 2));
	Matrix4d omega;
	omega << 0, 0.5 * r, -0.5 * q, 0.5 * p,
		     -0.5 * r, 0, 0.5 * p, 0.5 * q,
		     0.5 * q, -0.5 * p, 0, 0.5 * r,
		     -0.5 * p, -0.5 * q, -0.5 * r, 0;
	if(sigma != 0) {
		double sigma_cos = cos(sigma / 2);
		double sigma_sin = sin(sigma / 2);
		quaternion_ = sigma_cos * quaternion_ + 2 / sigma * sigma_sin * omega * quaternion_;
		quaternion_ = 1 / (quaternion_.norm()) * quaternion_;
	}
}

void Inertial::ConvertQuaternionToDcm() {
	double p1 = pow(quaternion_(0), 2), p2 = pow(quaternion_(1), 2);
	double p3 = pow(quaternion_(2), 2), p4 = pow(quaternion_(3), 2);
	double p5 = p2 + p3;
    double p6 = 0.0;
	if (p1 + p4 + p5 != 0) {
		p6 = 2 / (p1 + p4 + p5);
	}

	dcm_matrix_ << 1 - p6 * p5, 0, 0,
				   0, 1 - p6 * ( p1 + p3), 0,
				   0, 0, 1 - p6 * (p1 + p2);

	// Type p1_new = p6 * quaternion(0);
	// Type p2_new = p6 * quaternion(1);
	// Type p5_new = p6 * quaternion(2) * quaternion(3);
	// Type p6_new = p1 * quaternion(1);
	// dcm_matrix(0, 1) = p6_new - p5_new;
	// dcm_matrix(1, 0) = p6_new + p5_new;
	// Type p5_new_new = p2_new * quaternion(3);
	// Type p6_new_new = p1_new * quaternion(2);
	// dcm_matrix(0, 2) = p6_new_new + p5_new_new;
	// dcm_matrix(2, 0) = p6_new_new - p5_new_new;
	// Type p5_new_new_new = p1_new * quaternion(3);
	// Type p6_new_new_new = p2_new * quaternion(2);
	// dcm_matrix(1, 2) = p6_new_new_new - p5_new_new_new;
	// dcm_matrix(2, 1) = p6_new_new_new + p5_new_new_new;
	p1 = p6 * quaternion_(0);
	p2 = p6 * quaternion_(1);
	p5 = p6 * quaternion_(2) * quaternion_(3);
	p6 = p1 * quaternion_(1);
	dcm_matrix_(0, 1) = p6 - p5;
	dcm_matrix_(1, 0) = p6 + p5;
	p5 = p2 * quaternion_(3);
	p6 = p1 * quaternion_(2);
	dcm_matrix_(0, 2) = p6 + p5;
	dcm_matrix_(2, 0) = p6 - p5;
	p5 = p1 * quaternion_(3);
	p6 = p2 * quaternion_(2);
	dcm_matrix_(1, 2) = p6 - p5;
	dcm_matrix_(2, 1) = p6 + p5;
}

void Inertial::ConvertDcmToQuaternion() {
	double T, S, qw, qx, qy, qz;
	Vector4d dd;
	T = 1 + dcm_matrix_(0, 0) + dcm_matrix_(1, 1) + dcm_matrix_(2, 2);

	if ( T > 1e-8) {
		S = 0.5 / sqrt(T);
		qw = 0.25 / S;
		qx = (dcm_matrix_(2, 1) - dcm_matrix_(1, 2)) * S;
		qy = (dcm_matrix_(0, 2) - dcm_matrix_(2, 0)) * S;
		qz = (dcm_matrix_(1, 0) - dcm_matrix_(0, 1)) * S;
		quaternion_ << qx, qy, qz, qw;
		return;
	}

	if (dcm_matrix_(0, 0) > dcm_matrix_(1, 1) && dcm_matrix_(0, 0) > dcm_matrix_(2, 2)) {
		S = sqrt(1 + dcm_matrix_(0, 0) - dcm_matrix_(1, 1) - dcm_matrix_(2, 2)) * 2;
		qw = (dcm_matrix_(2, 1) - dcm_matrix_(1, 2)) / S;
		qx = 0.25 * S;
		qy = (dcm_matrix_(0, 1) + dcm_matrix_(1, 0)) / S;
		qz = (dcm_matrix_(0, 2) + dcm_matrix_(2, 0)) / S;
	}
	else if (dcm_matrix_(1, 1) > dcm_matrix_(2, 2)) {
		S = sqrt(1 + dcm_matrix_(1,1) - dcm_matrix_(0, 0) - dcm_matrix_(2, 2)) * 2;
		qw = (dcm_matrix_(0, 2) - dcm_matrix_(2, 0)) / S;
		qx = (dcm_matrix_(0, 1) + dcm_matrix_(1, 0)) / S;
		qy = 0.25 * S;
		qz = (dcm_matrix_(1, 2) + dcm_matrix_(2, 1)) / S;
	}
	else {
		S = sqrt(1 + dcm_matrix_(2, 2) - dcm_matrix_(0, 0) - dcm_matrix_(1, 1)) * 2;
		qw = (dcm_matrix_(1, 0) - dcm_matrix_(0, 1)) / S;
		qx = (dcm_matrix_(0, 2) + dcm_matrix_(2, 0)) / S;
		qy = (dcm_matrix_(1, 2) + dcm_matrix_(2, 1)) / S;
		qz = 0.25 * S;
	}
	quaternion_ << qx, qy, qz, qw;
}

void Inertial::ConvertEulerToDcm() {
	double cr, sr, cp, sp, cy, sy;
	cr = cos(state_(6));
	sr = sin(state_(6));
	cp = cos(state_(7));
	sp = sin(state_(7));
	cy = cos(state_(8));
	sy = sin(state_(8));
	dcm_matrix_ << cy * cp, sy * cp, -sp,
		          -sy * cr + cy * sp * sr, cy * cr + sy * sp * sr, cp * sr,
		          sy * sr + cy * sp * cr, -cy * sr + sy * sp * cr, cp * cr;
}

} // namespace inertial


