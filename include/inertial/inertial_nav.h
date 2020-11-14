// Authors: jiananz1@uci.edu

#pragma once

#include "common_include.h"
#include "state.h"

namespace inertial {

class Inertial final {
public:
	typedef cl::StateVector<9>::State State;
	typedef cl::StateVector<9>::Covariance Covariance;
	typedef Eigen::Matrix<double, 3, 6> MatrixTbS;
	typedef Eigen::Matrix<double, 9, 6> MatrixNbS;
	typedef Eigen::Matrix<double, 4, 4> MatrixFbF;
	typedef Eigen::Matrix<double, 6, 6> MatrixSbS;
	typedef Eigen::Matrix<double, 9, 3> MatrixNbT;
	typedef Eigen::Matrix<double, 3, 9> MatrixTbN;
	typedef Eigen::Matrix<double, 4, 9> MatrixFbN;
	typedef Eigen::Matrix<double, 9, 4> MatrixNbF;

    Inertial(double roll, double pitch);
    ~Inertial() = default;
	Inertial & operator=(const Inertial & object) = default;

    // Free INS:
	// Input: 
	//		 readouts_acc: the readouts from accelerometer
	//       readouts_gyro: the readouts from gyroscope
	void Propagate(const Eigen::Vector3d& readouts_acc, const Eigen::Vector3d& readouts_gyro, double dt);

	// If the current step zero velocity based on the given threshold (look at the magnitude of 
	// angular velocity comparing with threshold value at given window).
	bool IsZeroVelocity(const Eigen::Vector3d& readouts_gyro);
	
    // Update the free INS with the zero velocity psuedo measurement
	void UpdateZeroVelocity();

    void InitiateFilter();

	void InitializeQuaternionAndDcm() {
		ConvertEulerToDcm();
		ConvertDcmToQuaternion();
	}

	const State& state() const {
		return state_;
	}

	const Covariance& covariance() const {
		return covariance_;
	}

	void PrintState() const {
		std::cout << state_.transpose() << std::endl;
	}

	void SaveState(std::ofstream & of) const {
		of << state_.transpose() << " " << covariance_(0) << " " << covariance_(1) << " " << covariance_(2) << std::endl;
	}

private:
    // Input:
	//       readouts_acc: accelorometer readouts from IMU
	//       readouts_gyro: gyroscope readouts from IMU
	//       dt: step size between two readouts.
	// Output:
	//        quaternion: 4 * 1 vector (float type for now);
	void UpdateQuaternion(const Eigen::Vector3d & readouts_gyro, double dt);
	// Ipute: quaternion
	// OUtput: DCM matrix
	void ConvertQuaternionToDcm();
	void ConvertDcmToQuaternion();
	void ConvertEulerToDcm();

    State state_;
    Covariance covariance_;
    MatrixSbS covariance_imu_;
    Eigen::Vector4d quaternion_;
    Eigen::Matrix3d dcm_matrix_;

	// Zero velocity detector configs
	int window_size_;
	double threshold_;
	double sigma_;
	Eigen::Matrix3d covariance_zupt_;
};

} // namespace

