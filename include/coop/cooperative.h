// Authors: jiananz1@uci.edu

#pragma once

#include <unordered_map>

#include "common_include.h"
#include "coop/connector.h"
#include "coop/measurement_model.h"
#include "state.h"

/*
 * Define the class for cooperative localization
 * Cooperative is define as an ABC with interface implementation,
 * an implementation of DMV method has realized
 * TODO: realize PECMV method
 */

namespace cl {
namespace coop {

class CooperativeImu {
public:
	typedef ImuPlusRange::MeasurementClass MeasurementClass;
    typedef ImuPlusRange::Measurement Measurement;
	typedef ImuPlusRange::StateClass StateClass;
	typedef ImuPlusRange::State State;
	typedef ImuPlusRange::Jacobian Jacobian;
	typedef ImuPlusRange::MeasurementCovariance MeasurementCovariance;

	CooperativeImu();
	~CooperativeImu() = default;

	void set_state(const State& state) {
		state_ = state;
	}

	void set_measurement(const Measurement& measurement) {
		measurement_ = measurement;
	}

	


	// Cooperative(Type s_a, 
	// 		 	Type s_g, 
	// 		 	Vector9d init_s)
	// 		 	: Inertial<double>(s_a, s_g, init_s), is_success(false) {}
	// ~Cooperative() {}

	// void CalculateUpdate(const Vector3d & pose_relative, Type x, Type y, Type z, Type angle);
	// // Inertialize the serial port communication
	// // Input:
	// //       serial_port: the name of the serial_port
	// // Output:
	// //        returned serial communication number fd
	// int InitializeSerialCommunication(const char * serial_port);
	// void WriteToSerial(int fd, Type state_x_prior, Type state_y_prior, Type uncertainty_prior);
	// void ReadFromSerial(int fd);
	// void DecodeSerialData(const char * buf, int num);
	// bool IsCommunicationSuccess() {
	// 	return is_success;
	// }
	// Cooperative & operator=(const Cooperative & object);

private:
    ImuPlusRange::Ptr measurement_mode_ptr_;
	Connector::Ptr connector_ptr_;
	Measurement measurement_;
	State state_;
	std::unordered_map<std::string, int> address_to_id_agent_;
	std::unordered_map<std::string, int> address_to_id_beacon_;
};

} // namespace coop
} // namespace cl
