// Authors: jiananz1@uci.edu

#pragma once

#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "common_include.h"

namespace inertial {

class Inertial final {
public:
    typedef Matrix<Type, 9, 9> MatrixNbN;
	typedef Matrix<Type, 3, 6> MatrixTbS;
	typedef Matrix<Type, 9, 6> MatrixNbS;
	typedef Matrix<Type, 9, 1> Vector9d;
	typedef Matrix<Type, 4, 4> MatrixFbF;
	typedef Matrix<Type, 6, 6> MatrixSbS;
	typedef Matrix<Type, 9, 3> MatrixNbT;
	typedef Matrix<Type, 3, 9> MatrixTbN;
	typedef Matrix<Type, 4, 9> MatrixFbN;
	typedef Matrix<Type, 9, 4> MatrixNbF;

    Inertial();
    ~Inerial() = default;

    // Free INS:
	// Input: 
	//		 readouts_acc: the readouts from accelerometer
	//       readouts_gyro: the readouts from gyroscope
	void Propagate(const Vector3d & readouts_acc, const Vector3d & readouts_gyro, double dt);

	// If the current step zero velocity based on the given threshold (look at the magnitude of 
	// angular velocity comparing with threshold value at given window).
	bool CheckZeroVelocity(int window, double sigma_g,double threshold, const Vector3d & readouts_gyro);
	
    // Update the free INS with the zero velocity psuedo measurement
	void UpdateZeroVelocity();

    void InitiateFilter();

	void InitializeQuaternionAndDcm() {
		ConvertEulerToDcm();
		ConvertDcmToQuaternion();
	}

	Vector9d & state() {
		return state;
	}

	MatrixNbN & covariance() {
		return covariance;
	}

	void PrintState() {
		cout << state.transpose() << endl;
	}

	void SaveState(std::ofstream & of) {
	of << state.transpose() <<" " << acc_.transpose() << " " << gyro_.transpose() <<covariance(0) << " " << covariance(1) << " " << covariance(2) << endl;
	}

	// Overloaded assignment operator
	Inertial & operator=(const Inertial & object);
	// Function to save the data to local file
	//friend std::ofstream & operator<<(std::ofstream & of, const Inertial<Type> &);

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
};

} // namespace
