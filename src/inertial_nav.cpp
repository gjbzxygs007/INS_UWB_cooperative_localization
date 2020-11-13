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
    variance_position *= variance_position;
    variance_speed *= variance_speed;
    variance_attitude *= variance_attitude;

    state_ = Vector9d::Zeros();
    state_(6) = roll;
    state_(7) = pitch;
    state_(8) = cl::Config::get<double>("heading_angle");

    covariance_ << 1e-10, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 1e-10, 0, 0, 0, 0, 0, 0, 0,
				    0, 0, 1e-10, 0, 0, 0, 0, 0, 0,
		 		    0, 0, 0, 1e-10, 0, 0, 0, 0, 0,
				    0, 0, 0, 0, 1e-10, 0, 0, 0, 0,
				    0, 0, 0, 0, 0, 1e-10, 0, 0, 0,
					0, 0, 0, 0, 0, 0, pow(0.1 * kPi / 180, 2), 0, 0,
					0, 0, 0, 0, 0, 0, 0, pow(0.1 * kPi / 180, 2), 0,
					0, 0, 0, 0, 0, 0, 0, 0, pow(0.1 * kPi / 180, 2);

    covariance_imu_ << pow(sigma_acc,2), 0, 0, 0, 0, 0,
		 			   0, pow(sigma_acc,2), 0, 0, 0, 0,
		 			   0, 0, pow(sigma_acc,2), 0, 0, 0,
		               0, 0, 0, pow(sigma_gyro,2), 0, 0,
	                   0, 0, 0, 0, pow(sigma_gyro,2), 0,
                       0, 0, 0, 0, 0, pow(sigma_gyro,2);
    ConvertEulerToDcm();
}




void Inertial::ConvertQuaternionToDcm() {
	double p1 = pow(quaternion_(0), 2), p2 = pow(quaternion_(1), 2);
	double p3 = pow(quaternion_(2), 2), p4 = pow(quaternion_(3), 2);
	double p5 = p2 + p3
    double p6 = 0;
	if (p1 + p4 + p5 != 0) {
		p6 = 2 / (p1 + p4 + p5);
	}

	dcm_matrix << 1 - p6 * p5, 0, 0,
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


