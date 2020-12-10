// Authors: jiananz1@uci.edu

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "common_include.h"
#include "config.h"
#include "coop/connector.h"
// #include "coop/cooperative.h"
#include "inertial/inertial_nav.h"
#include "vn/compositedata.h"
#include "vn/thread.h"
#include "vn/sensors.h"


using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace std::chrono;

namespace {

// VnSensor vn_sensor;
// MagneticAccelerationAndAngularRatesRegister init;
// CompositeData comp;
// long long now, now_prev;
// inertial::Inertial filter;
// // int fd_r = filter_cl.InitializeSerialCommunication("/dev/ttyACM1");
// // int fd_s = filter_cl.InitializeSerialCommunication("/dev/ttyACM0");
// std::ofstream of;
// Vector3d pose_relative;
// void asciiOrBinaryAsyncInit(void* userData, Packet& p, size_t index);
// void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);
// void NavWithoutUpdate(Inertial& filter, vec3f accel, vec3f gyro, double dt);
// void NavWithUpdate(Cooperative<double> & filter, vec3f accel, vec3f gyro, int i, double dt);

} // namespace

int main(int argc, char *argv[]) {
    if ( argc != 2 ) {
        std::cout <<" Usage: run_cl parameter_file" << std::endl;
        return 1;
    }


    cl::Config::SetParameterFile(argv[1]);

        auto n = cl::Config::get<cv::FileNode>("addresses_of_agents");                       // Read string sequence - Get node
        if (n.type() != cv::FileNode::SEQ)
        {
            std::cerr << "strings is not a sequence! FAIL" << std::endl;
            return 1;
        }
        cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
        for (; it != it_end; ++it)
            std::cout << (std::string)*it << std::endl;

    std::string path = cl::Config::get<std::string>("uwb_path");
    std::cout << path << std::endl;
	return 0;
}
	
//     cout << sensor_port << endl;
// 	const uint32_t SensorBaudrate = 115200;
	
// 	// Now let's create a VnSensor object and use it to connect to our sensor.
// 	vn_sensor.connect(SensorPort, SensorBaudrate);

// 	// Let's query the sensor's model number.
// 	string mn = vn_sensor.readModelNumber();
// 	cout << "Model Number: " << mn << endl;
// 	of.open("results.dat");
// 	cout << "Initializing the filters ..." << endl;
// 	int div = 4;
// 	BinaryOutputRegister bor(
// 		ASYNCMODE_PORT1,
// 		div,
// 		COMMONGROUP_NONE,	// Note use of binary OR to configure flags.
// 		TIMEGROUP_NONE,
// 		IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE,
//     	GPSGROUP_NONE,
// 		ATTITUDEGROUP_NONE,
// 		INSGROUP_NONE,
//     	GPSGROUP_NONE);

// 	vn_sensor.writeBinaryOutput1(bor);
// 	std::thread t_input(WaitForInput);
// 	t_input.detach();
// 	// std::thread t_input_2(WaitForInput2);
// 	// t_input_2.detach();
// 	// std::thread t_input_3(WaitForInput3);
// 	// t_input_3.detach();

// 	vn_sensor.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncInit);
// 	Thread::sleepSec(10);
// 	u1 = u1 / (count_init - 1000);
// 	u2 = u2 / (count_init - 1000);
// 	u3 = u3 / (count_init - 1000);
// 	vn_sensor.unregisterAsyncPacketReceivedHandler();
// 	cout << count_init << endl;

// 	Inertial<double>::Vector9d state_initial;
// 	double roll, pitch, yaw;
// 	roll = atan2(-u2, -u3);
// 	pitch = atan2(u1, sqrt(pow(u2, 2) + pow(u3, 2)));
// 	yaw = 0;

// 	state_initial << 0, 0, 0, 0, 0, 0, roll, pitch, yaw; 
// 	cout << state_initial.transpose() << endl;

// 	filter = Inertial<double>(0.5, 0.0087266, state_initial);
// 	filter_cl = Cooperative<double>(0.5, 0.0087266, state_initial);
// 	filter.InitializeQuaternionAndDcm();
// 	filter_cl.InitializeQuaternionAndDcm();
// 	now_prev = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
// 	vn_sensor.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);

// 	Thread::sleepSec(3600);
// 	cout << "Starting sleep..." << endl;
// 	vn_sensor.unregisterAsyncPacketReceivedHandler();

// 	vn_sensor.disconnect();

// 	return 0;
// }

// void asciiOrBinaryAsyncInit(void* userData, Packet& p, size_t index) {
// 	if (p.type() == Packet::TYPE_BINARY) {
// 		// First make sure we have a binary packet type we expect since there
// 		// are many types of binary output types that can be configured.
// 		if (!p.isCompatible(
// 			COMMONGROUP_NONE,
// 			TIMEGROUP_NONE,
// 			IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE,
//       		GPSGROUP_NONE,
// 			ATTITUDEGROUP_NONE,
// 			INSGROUP_NONE,
//       		GPSGROUP_NONE)) return;
//       // Not the type of binary packet we are expecting.
// 		count_init++;
// 		if (count_init <= 1000) {
// 			return;
// 		}
// 		vec3f accel;
// 		comp.parse(p,comp);
// 		accel = comp.anyAcceleration();
// 		u1 += accel[0];
// 		u2 += accel[1];
// 		u3 += accel[2];	
// 	}	
// }


// void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
// {	
// 	if (p.type() == Packet::TYPE_BINARY) {
// 		// First make sure we have a binary packet type we expect since there
// 		// are many types of binary output types that can be configured.
// 		if (!p.isCompatible(
// 			COMMONGROUP_NONE,
// 			TIMEGROUP_NONE,
// 			IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE,
//       		GPSGROUP_NONE,
// 			ATTITUDEGROUP_NONE,
// 			INSGROUP_NONE,
//       		GPSGROUP_NONE))
//       // Not the type of binary packet we are expecting.
// 			return;

// 		now = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
// 		double dt = (double)(now - now_prev) / 1000;
// 		vec3f accel;
// 		vec3f gyro;
// 		comp.parse(p,comp);
// 		accel = comp.anyAcceleration();
// 		gyro = comp.anyAngularRate();
// 		comp.reset();
// 		int i = 0;
// 		// if (!is_corrected) {
// 		// 	dt = 0.97 * dt;
// 		// }
// 		std::thread t1(NavWithoutUpdate, std::ref(filter), accel, gyro, dt);
// 		std::thread t2(NavWithUpdate, std::ref(filter_cl), accel, gyro, i, dt);
// 		t1.join();
// 	    t2.join();

// 		// cout << now << endl;
// 		filter_cl.PrintState();
// 		of << dt;
// 		filter_cl.SaveState(of);
// 		if (counts % 5 == 0) {
// 			filter_cl.WriteToSerial(fd_s, filter.AccessToState()(0), filter.AccessToState()(1), filter.AccessToCovariance()(0, 0) + filter.AccessToCovariance()(1, 1));
// 		}
// 		counts++;
// 		// cout << dt <<endl;
// 		// cout << accel << endl;
// 		// cout << gyro <<endl;
// 		now_prev = now;
// 	}
// }

// void NavWithUpdate(Cooperative<double> &filter, vec3f accel, vec3f gyro, int i, double dt) {
// 	Vector3d readouts_acc;
// 	Vector3d readouts_gyro;
// 	readouts_acc << accel[0], accel[1], accel[2];
// 	readouts_gyro << gyro[0], gyro[1], gyro[2];
// 	filter.Propagation(readouts_acc, readouts_gyro, dt);
// 	if (filter.IsZeroVelocity(1, 0.1, 0.3e5, readouts_gyro)) {
// 		filter.UpdateZeroVelocity();
// 		if (is_update_needed_1) {
// 	    	cout << "Update" << endl;
// 	    	filter.ReInitiate();
// 	    	pose_relative << 3.0, -2.9, 0.0;
// 	    	filter.CalculateUpdate(pose_relative, 3.0, -2.9, 0.0, 3.14);
// 	    	is_update_needed_1 = false;
// 	    	return;
// 	    }

// 	    if (is_update_needed_2) {
// 	    	cout << "Update" << endl;
// 	    	filter.ReInitiate();
// 	    	pose_relative << 0.5, -2.8, 0.0;
// 	    	filter.CalculateUpdate(pose_relative, 0.5, -2.8, 0.0, 3.14);
// 	    	is_update_needed_2 = false;
// 	    	return;
// 	    }

// 	    if (is_update_needed_3) {
// 	    	cout << "Update" << endl;
// 	    	filter.ReInitiate();
// 	    	pose_relative << 0.5, -1.0, 0.0;
// 	    	filter.CalculateUpdate(pose_relative, 0.5, -1.0, 0.0, 3.14159265 / 2);
// 	    	is_update_needed_3 = false;
// 	    	return;
// 	    }
// 	}
// 	filter.ReadFromSerial(fd_r);
// }



// void NavWithoutUpdate(Inertial<double> &filter, vec3f accel, vec3f gyro, double dt)
// {
// 	Vector3d readouts_acc;
// 	Vector3d readouts_gyro;
// 	readouts_acc << accel[0], accel[1], accel[2];
// 	readouts_gyro << gyro[0], gyro[1], gyro[2];
// 	filter.Propagation(readouts_acc, readouts_gyro, dt);
// 	if (filter.IsZeroVelocity(1, 0.1, 0.3e5, readouts_gyro)) {
// 		filter.UpdateZeroVelocity();
// 		cout << "zup" << endl;
// 	}
// }

// void WaitForInput() {
// 	char ch;
// 	cout << "Wait for the start of CL!" << endl;
// 	cout << "Start by input '1'!" << endl;
// 	while (1) {
// 		std::cin >> ch;
// 		if (ch == '1') {
// 			is_update_needed_1 = true;
// 		}
// 		else if (ch == '2') {
// 			is_update_needed_2 = true;
// 		}
// 		else if (ch == '3') {
// 			is_update_needed_3 = true;
// 		}
// 	}
	
// }

// void WaitForInput2() {
// 	char ch;
// 	// cout << "Wait for the start of CL!" << endl;
// 	cout << "Start by input '2'!" << endl;
// 	while (1) {
// 		std::cin >> ch;
// 		if (ch == '2') {
// 			break;
// 		}
// 	}
// 	is_update_needed_2 = true;
// }

// void WaitForInput3() {
// 	char ch;
// 	// cout << "Wait for the start of CL!" << endl;
// 	cout << "Start by input '3'!" << endl;
// 	while (1) {
// 		std::cin >> ch;
// 		if (ch == '3') {
// 			break;
// 		}
// 	}
// 	is_update_needed_3 = true;
// }

