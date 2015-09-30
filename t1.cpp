#include <fcntl.h>
#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

struct TRoboClaw {
	struct TRoboClawException : public std::exception {
		std::string s;
		TRoboClawException(std::string ss) : s(ss) {}
		~TRoboClawException() throw() {}
		const char* what() const throw() { return s.c_str(); }
	};

	int clawPort;
	static const char portAddress = 0x80;
	static const int MAX_COMMAND_RETRIES = 5;
	static const bool DEBUG = true;

	TRoboClaw() {
		clawPort = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (clawPort == -1) {
			cout << "[TRoboClaw constructor] unable to open USB port";
			throw new TRoboClawException("Unable to open USB port");
		}

		struct flock lock;
		lock.l_type = F_WRLCK;
		lock.l_whence = SEEK_SET;
		lock.l_start = 0;
		lock.l_len = 0;
		lock.l_pid = getpid();
		if (fcntl(clawPort, F_SETLK, &lock) != 0) {
			cout << "[TRoboClaw] Device is already locked" << endl;
			throw new TRoboClawException("[TRoboClaw] Device is already locked");
		}

        // Set up the port so reading nothing returns immediately, instead of blocking
		//fcntl(clawPort, F_SETFL, FNDELAY);

        // Fetch the current port settings
		struct termios portOptions;
		tcgetattr(clawPort, &portOptions);
		memset(&portOptions.c_cc, 0, sizeof(portOptions.c_cc));

        // Flush the port's buffers (in and out) before we start using it
        tcflush(clawPort, TCIOFLUSH);

        // Set the input and output baud rates
        cfsetispeed(&portOptions, B115200);
        cfsetospeed(&portOptions, B115200);

        // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
        //   this program from "owning" the port and to enable receipt of data.
        //   Also, it holds the settings for number of data bits, parity, stop bits,
        //   and hardware flow control. 
        portOptions.c_cflag = CS8 | CLOCAL | CREAD;
        portOptions.c_iflag = IGNPAR;
        portOptions.c_oflag = 0;
        portOptions.c_lflag = 0;
        // portOptions.c_cflag |= CLOCAL;
        // portOptions.c_cflag |= CREAD;
        // // Set up the frame information.
        // portOptions.c_cflag &= ~PARENB;
        // portOptions.c_cflag &= ~CSTOPB;
        // portOptions.c_cflag &= ~CSIZE;
        // portOptions.c_cflag |= CS8;
        // portOptions.c_iflag &= ~(ICANON | ECHO |ECHOE | ECHOK | ECHONL | ISIG | IEXTEN | INLCR | IGNCR | ICRNL | IGNBRK | IUCLC | PARMRK | INPCK | ISTRIP | PARENB | PARODD);
        // portOptions.c_oflag &= ~(OPOST | ONLCR | OCRNL);

        // Now that we've populated our options structure, let's push it back to the
        //   system.
        tcsetattr(clawPort, TCSANOW, &portOptions);

        // Flush the buffer one more time.
        tcflush(clawPort, TCIOFLUSH);
        usleep(200000);
		if (DEBUG) cout << "[TRoboClaw constructor] USB open\n";
	}

	uint8_t readByteWithTimeout() {
		fd_set	set;
		struct timeval timeout;
		int selectResult;

		FD_ZERO(&set); // Clear the set.
		FD_SET(clawPort, &set); // Add file descriptor to the set.
		timeout.tv_sec = 0;
		timeout.tv_usec = 10000; // 10 milliseconds.
		selectResult = select(FD_SETSIZE/*clawPort + 1*/, &set /* read */, NULL /* write */, NULL /* exception */, &timeout);
		if (DEBUG) cout << "[readByteWithTimeout] SELECT clawPort: " << clawPort << ", FD_SETSIZE: " << FD_SETSIZE << " errno: " << errno << ", selectResult: " << selectResult << endl;//#####
		if (selectResult == -1) {
			if (DEBUG) cout << "[readWithTimeout] ERROR " << errno << endl;
			return 0;
		} else if (selectResult == 0) {
			if (DEBUG) cout << "[readWithTimeout] TIMEOUT " << endl;
			return 0;
		} else {
			char buffer[1];
			int bytesRead = read(clawPort, buffer, sizeof(buffer));
			if (bytesRead != 1) { throw TRoboClawException("Failed to read 1 byte"); }
			if (DEBUG) cout << "[readByteWithTimeout] ...byte: " << hex << int(buffer[0]) << dec << endl;
			return buffer[0];
		}
	}

	void writeByte(uint8_t byte) {
		ssize_t result = write(clawPort, &byte, 1);
		if (result != 1) {
			throw new TRoboClawException("Unable to write one byte");
		} else { if (DEBUG) cout << "[writeByte] WRITING: " << hex << int(byte) << dec << endl; }
	}

	bool writeN(bool sendChecksum, uint8_t cnt, ...) {
		uint8_t crc = 0;
		va_list marker;
		va_start(marker, cnt);
		for (uint8_t i = 0; i < cnt; i++) {
			uint8_t byte = va_arg(marker, int);
			crc += byte;
			writeByte(byte);
		}

		va_end(marker);
		if (sendChecksum) writeByte(crc & 0x7F);
		return false;
	}

	unsigned short get2ByteCommandResult(uint8_t command) {
		int retry;
		for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
			try {
				uint8_t checkSum = portAddress + command;

				writeN(false, 2, portAddress, command);
				unsigned short result = 0;
				uint8_t datum = readByteWithTimeout();
				checkSum += datum;
				result |= datum << 8;
				datum = readByteWithTimeout();
				checkSum += datum;
				result |= datum;

				uint8_t responseChecksum = readByteWithTimeout();
				if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
					if (DEBUG) cout << "[get2ByteCommandResult] Expected checkSum of: " << hex << int(checkSum) << ", but got:" << int(responseChecksum) << dec << endl;
					throw new TRoboClawException("[get2ByteCommandResult] INVALID CHECKSUM");
				}

				return result;
			} catch (TRoboClawException e) {
				if (DEBUG) cout << "[get2ByteCommandResult] Exception: " + *e.what() << ", retry number: " << retry << endl;
			}
		}

		cout << "[get2ByteCommandResult] RETRY COUNT EXCEEDED" << endl;
		throw new TRoboClawException("[get2ByteCommandResult] RETRY COUNT EXCEEDED");
	}

	unsigned long getUlongCommandResult(uint8_t command) {
		int retry;
		for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
			try {
				uint8_t checkSum = portAddress + command;

				writeN(false, 2, portAddress, command);
				unsigned long result = 0;
				uint8_t datum = readByteWithTimeout();
				checkSum += datum;
				result |= datum << 24;
				datum = readByteWithTimeout();
				checkSum += datum;
				result |= datum << 16;
				datum = readByteWithTimeout();
				checkSum += datum;
				result |= datum << 8;
				datum = readByteWithTimeout();
				checkSum += datum;
				result |= datum;

				uint8_t responseChecksum = readByteWithTimeout();
				if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
					if (DEBUG) cout << "[getUlongCommandResult] Expected checkSum of: " << hex << int(checkSum) << ", but got:" << int(responseChecksum) << dec << endl;
					throw new TRoboClawException("[getUlongCommandResult] INVALID CHECKSUM");
				}

				return result;
			} catch (TRoboClawException e) {
				if (DEBUG) cout << "[getUlongCommandResult] Exception: " + *e.what() << ", retry number: " << retry << endl;
			}
		}

		cout << "[get2ByteCommandResult] RETRY COUNT EXCEEDED" << endl;
		throw new TRoboClawException("[get2ByteCommandResult] RETRY COUNT EXCEEDED");
	}

	typedef struct {
		unsigned long p1;
		unsigned long p2;
	} ULongPair;

	ULongPair getULongPairCommandResult(uint8_t command) {
		int retry;
		for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
			try {
				uint8_t checkSum = portAddress + command;

				writeN(false, 2, portAddress, command);
				unsigned long result1 = 0;
				uint8_t datum = readByteWithTimeout();
				checkSum += datum;
				result1 |= datum << 24;
				datum = readByteWithTimeout();
				checkSum += datum;
				result1 |= datum << 16;
				datum = readByteWithTimeout();
				checkSum += datum;
				result1 |= datum << 8;
				datum = readByteWithTimeout();
				checkSum += datum;
				result1 |= datum;

				unsigned long result2 = 0;
				datum = readByteWithTimeout();
				checkSum += datum;
				result2 |= datum << 24;
				datum = readByteWithTimeout();
				checkSum += datum;
				result2 |= datum << 16;
				datum = readByteWithTimeout();
				checkSum += datum;
				result2 |= datum << 8;
				datum = readByteWithTimeout();
				checkSum += datum;
				result2 |= datum;				uint8_t responseChecksum = readByteWithTimeout();
				if ((checkSum & 0x7F) != (responseChecksum & 0x7F)) {
					if (DEBUG) cout << "[getUlongPairCommandResult] Expected checkSum of: " << hex << int(checkSum) << ", but got:" << int(responseChecksum) << dec << endl;
					throw new TRoboClawException("[getULongPairCommandResult] INVALID CHECKSUM");
				}

				ULongPair result;
				result.p1 = result1;
				result.p2 = result2;
				return result;
			} catch (TRoboClawException e) {
				if (DEBUG) cout << "[getULongPairCommandResult] Exception: " + *e.what() << ", retry number: " << retry << endl;
			}
		}

		cout << "[getULongPairCommandResult] RETRY COUNT EXCEEDED" << endl;
		throw new TRoboClawException("[getULongPairCommandResult] RETRY COUNT EXCEEDED");
	}

	float getLogicBatteryLevel() {
		return ((float) get2ByteCommandResult(25)) / 10.0;
	}

	float getM1MaxCurrent() {
		ULongPair temp = getULongPairCommandResult(135);
		return ((float) temp.p1) / 100.0;
	}

	string getVersion() {
		writeN(false, 2, portAddress, 21);
		string result;
		unsigned char b;

		do {
			b = readByteWithTimeout();
			if (b == '\0') break;
			result += b;
		} while(b);

		b = readByteWithTimeout(); // CRC
		return result;
	}


};

int main(int argc, char **argv) {
	cout << "Hello world\n";
	TRoboClaw roboClaw;
	float floatVal;

	string stringVal = roboClaw.getVersion();
	cout << "VERSION: " << stringVal << endl;
	floatVal = roboClaw.getLogicBatteryLevel();
	cout << "LOGIC VOLTAGE: " << floatVal << endl;
	floatVal = roboClaw.getM1MaxCurrent();
	cout << "MAX M1 Motor Current: " << floatVal << endl;
}
