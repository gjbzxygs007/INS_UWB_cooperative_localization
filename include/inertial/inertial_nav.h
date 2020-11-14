// Authors: jiananz1@uci.edu

#pragma once

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "common_include.h"
#include "coop/measurement_model.h"
#include "state.h"

namespace inertial {

class Inertial final {
public:
	typedef 

	typedef Matrix<double, 3, 6> MatrixTbS;
	typedef Matrix<double, 9, 6> MatrixNbS;
	typedef Matrix<double, 9, 1> Vector9d;
	typedef Matrix<double, 4, 4> MatrixFbF;
	typedef Matrix<double, 6, 6> MatrixSbS;
	typedef Matrix<double, 9, 3> MatrixNbT;
	typedef Matrix<double, 3, 9> MatrixTbN;
	typedef Matrix<double, 4, 9> MatrixFbN;
	typedef Matrix<double, 9, 4> MatrixNbF;

    Inertial(double roll, double pitch);
    ~Inertial() = default;
	Inertial & operator=(const Inertial & object) = default;

    // Free INS:
	// Input: 
	//		 readouts_acc: the readouts from accelerometer
	//       readouts_gyro: the readouts from gyroscope
	void Propagate(const Vector3d& readouts_acc, const Vector3d& readouts_gyro, double dt);

	// If the current step zero velocity based on the given threshold (look at the magnitude of 
	// angular velocity comparing with threshold value at given window).
	bool IsZeroVelocity(const Vector3d& readouts_gyro);
	
    // Update the free INS with the zero velocity psuedo measurement
	void UpdateZeroVelocity();

    void InitiateFilter();

	void InitializeQuaternionAndDcm() {
		ConvertEulerToDcm();
		ConvertDcmToQuaternion();
	}

	const Vector9d& state() const {
		return state_;
	}

	const MatrixNbN& covariance() const {
		return covariance_;
	}

	void PrintState() const {
		cout << state_.transpose() << endl;
	}

	void SaveState(std::ofstream & of) const {
	of << state_.transpose() << " " << covariance_(0) << " " << covariance_(1) << " " << covariance_(2) << endl;
	}

private:
    // Input:
	//       readouts_acc: accelorometer readouts from IMU
	//       readouts_gyro: gyroscope readouts from IMU
	//       dt: step size between two readouts.
	// Output:
	//        quaternion: 4 * 1 vector (float type for now);
	void UpdateQuaternion(const Vector3d & readouts_gyro, double dt);
	// Ipute: quaternion
	// OUtput: DCM matrix
	void ConvertQuaternionToDcm();
	void ConvertDcmToQuaternion();
	void ConvertEulerToDcm();

    Vector9d state_;
    MatrixNbN covariance_;
    MatrixSbS covariance_imu_;
    Vector4d quaternion_;
    Matrix3d dcm_matrix_;

	// Zero velocity detector configs
	int window_size_;
	double threshold_;
	double sigma_;
	Matrix3d covariance_zupt_;
};

} // namespace

