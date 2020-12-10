// Authors: jiananz1@uci.edu

#include "coop/connector.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "common_include.h"

namespace cl {
namespace coop {

void Connector::Initialize(const char * serial_port) {
    struct termios toptions;
	fd_ = open(serial_port, O_RDWR | O_NOCTTY);

	if(fd_ == -1) {
  		printf( "Failed to open UWB port\n" );
	}

	printf("UWB opened as %i\n", fd_);

	sleep(1);

	int sett = tcgetattr(fd_, &toptions);
	printf("Current UWB port setting %i\n", sett);

	cfsetispeed(&toptions, B115200);
	cfsetospeed(&toptions, B115200);

	/* UART communication, 8 bits, no parity, no stop bits */
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	toptions.c_lflag = 0;
	toptions.c_cc[VTIME] = 0.1;// read return afte 1/1000 seconds
	toptions.c_cc[VMIN] = 0;

	/* commit the serial port settings */
	tcsetattr(fd_, TCSANOW, &toptions);
}

bool Connector::Write(const Message& msg) {
    int n, i;
	char * buf = new char[1023];
    char * buf_cur = buf;
    const char * id_char = msg.id.c_str();
    memcpy(buf_cur, id_char , msg.id.length());
    buf_cur += msg.id.length();
    for (double s : msg.state) {
        std::string temp = std::to_string(s) + ",";
        const char* temp_c = temp.c_str();
        memcpy(buf_cur, temp_c, temp.length());
        buf_cur += temp.length();
    }
    for (int i = 0; i < msg.covariance.size(); ++i) {
        for (int j = 0; j < msg.covariance[0].size(); ++j) {
            std::string temp = std::to_string(msg.covariance[i][j]) + ",";
            const char* temp_c = temp.c_str();
            memcpy(buf_cur, temp_c, temp.length());
            buf_cur += temp.length();
        }
    }
    std::string temp = std::to_string(msg.range) + ",";
    const char* temp_c = temp.c_str();
    memcpy(buf_cur, temp_c, temp.length());
    buf_cur += temp.length();
	n = write(fd_, buf, 1023);
    if(n < 0) {	
		return false;
	}
	delete [] buf;
    return true;
}


bool Connector::Read(Message * msg) {
    int n, i;
	char * buf = new char[1023];
	n = read(fd_, buf, 1000);
	buf[n] = '\0';
	// if (n != 0) {
	// 	std::cout << buf << std::endl;
	// }
	if(n < 0) {	
		return false;
	}
	bool is_parsed = Parse(buf, n, msg);
	delete [] buf;
    return is_parsed;
}

bool Connector::Parse(const char* buf, int num, Message* msg) {
    std::string delimiter = ",";
    std::string str(buf);
	std::size_t pos = 0;
    int count = 0;
    str += "'";
    int row = 0, col = 0;
	while ((pos = str.find(delimiter)) != std::string::npos) {
	    std::string token = str.substr(0, pos);
        if (count == 0) {
            msg->id = token;
            continue;
        }

	    try{
	    	std::stod(token);
	    }
	    catch(...){
	    	std::cout << "The message data is not a double!" << std::endl;
	    	return false;
	    }
        if (count < msg->state.size() + 1) {
            msg->state[count] = std::stod(token);
        }
        else if (count >= msg->state.size() + 1 && count < msg->state.size() +
                (msg->covariance.size() + 1) * msg->covariance.size() / 2 + 1) {
            msg->covariance[row][col] = std::stod(token);
            col++;
            if (col == msg->covariance.size()) {
                row++;
                col = row;
            }
        }
        else if (count == msg->state.size() + (msg->covariance.size() + 1) *
                msg->covariance.size() / 2 + 1) {
            msg->range = std::stod(token);
        }
        else {
            msg->power = std::stod(token);
        }
	    str.erase(0, pos + delimiter.length());
        count++;
	}
    if (count != msg->state.size() + (msg->covariance.size() + 1) * msg->covariance.size() / 2 + 2) {
        return false;
    }
    return true;
}

} // namespace coop
} // namespace cl