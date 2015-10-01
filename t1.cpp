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
#include <vector>

using namespace std;

struct TRoboClaw {
  enum {M1FORWARD = 0,
        M1BACKWARD = 1,
        SETMINMB = 2,
        SETMAXMB = 3,
        M2FORWARD = 4,
        M2BACKWARD = 5,
        M17BIT = 6,
        M27BIT = 7,
        MIXEDFORWARD = 8,
        MIXEDBACKWARD = 9,
        MIXEDRIGHT = 10,
        MIXEDLEFT = 11,
        MIXEDFB = 12,
        MIXEDLR = 13,
        GETM1ENC = 16,
        GETM2ENC = 17,
        GETM1SPEED = 18,
        GETM2SPEED = 19,
        RESETENC = 20,
        GETVERSION = 21,
        GETMBATT = 24,
        GETLBATT = 25,
        SETMINLB = 26,
        SETMAXLB = 27,
        SETM1PID = 28,
        SETM2PID = 29,
        GETM1ISPEED = 30,
        GETM2ISPEED = 31,
        M1DUTY = 32,
        M2DUTY = 33,
        MIXEDDUTY = 34,
        M1SPEED = 35,
        M2SPEED = 36,
        MIXEDSPEED = 37,
        M1SPEEDACCEL = 38,
        M2SPEEDACCEL = 39,
        MIXEDSPEEDACCEL = 40,
        M1SPEEDDIST = 41,
        M2SPEEDDIST = 42,
        MIXEDSPEEDDIST = 43,
        M1SPEEDACCELDIST = 44,
        M2SPEEDACCELDIST = 45,
        MIXEDSPEEDACCELDIST = 46,
        GETBUFFERS = 47,
        SETPWM = 48,
        GETCURRENTS = 49,
        MIXEDSPEED2ACCEL = 50,
        MIXEDSPEED2ACCELDIST = 51,
        M1DUTYACCEL = 52,
        M2DUTYACCEL = 53,
        MIXEDDUTYACCEL = 54,
        GETM1PID = 55,
        GETM2PID = 56,
        GETERROR = 90,
        WRITENVM = 94,
    	GETM1MAXCURRENT = 135};

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
		timeout.tv_usec = 100000; // 10 milliseconds.

		selectResult = select(clawPort + 1, &set /* read */, NULL /* write */, NULL /* exception */, &timeout);
		if (DEBUG) cout << "[readByteWithTimeout] SELECT clawPort: " << clawPort << ", FD_SETSIZE: " << FD_SETSIZE << " errno: " << errno << ", selectResult: " << selectResult << endl;//#####
		if (selectResult == -1) {
			if (DEBUG) cout << "[readByteWithTimeout] ERROR " << errno << endl;
			return 0;
		} else if (selectResult == 0) {
			if (DEBUG) cout << "[readByteWithTimeout] TIMEOUT " << endl;
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

	bool writeN(uint8_t cnt, ...) {
		va_list marker;
		va_start(marker, cnt);

		tcflush(clawPort, TCIOFLUSH);

		for (uint8_t i = 0; i < cnt; i++) {
			uint8_t byte = va_arg(marker, int);
			writeByte(byte);
		}

		va_end(marker);
		return false;
	}

	unsigned short get2ByteCommandResult(uint8_t command) {
		int retry;
		for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
			try {
				uint8_t checkSum = portAddress + command;

				writeN(2, portAddress, command);
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

				writeN(2, portAddress, command);
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

	uint32_t getLongCont(uint8_t& checksum) {
		uint32_t result = 0;
		uint8_t datum = readByteWithTimeout();
		checksum += datum;
		result |= datum << 24;
		datum = readByteWithTimeout();
		checksum += datum;
		result |= datum << 16;
		datum = readByteWithTimeout();
		checksum += datum;
		result |= datum << 8;
		datum = readByteWithTimeout();
		checksum += datum;
		result |= datum;
		return result;
	}

	ULongPair getULongPairCommandResult(uint8_t command) {
		int retry;
		for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
			try {
				uint8_t checksum = portAddress + command;

				writeN(2, portAddress, command);
				uint32_t result1 = getLongCont(checksum);
				uint32_t result2 = getLongCont(checksum);

				uint8_t responseChecksum = readByteWithTimeout();
				if ((checksum & 0x7F) != (responseChecksum & 0x7F)) {
					if (DEBUG) cout << "[getUlongPairCommandResult] Expected checksum of: " << hex << int(checksum) << ", but got:" << int(responseChecksum) << dec << endl;
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
		return ((float) get2ByteCommandResult(GETLBATT)) / 10.0;
	}

	float getM1MaxCurrent() {
		ULongPair temp = getULongPairCommandResult(GETM1MAXCURRENT);
		return ((float) temp.p1) / 100.0;
	}

	string getVersion() {
		writeN(2, portAddress, GETVERSION);
		string result;
		uint8_t b;

		do {
			b = readByteWithTimeout();
			if (b == '\0') break;
			result += b;
		} while(b);

		b = readByteWithTimeout(); // CRC
		return result;
	}

	#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg

	void setM1PID(float p, float i, float d, uint32_t qpps) {
		uint32_t kp = 13;//#####int(p * 65536.0);
		uint32_t ki = 27;//#####int(i * 65536.0);
		uint32_t kd = 59;//#####int(d * 65536.0);
		cout << "[setM1PID] p: " << hex << kp << ", i: " << ki << ", d: " << kd << ", qpps: " << qpps << dec << endl;
		writeN(18, portAddress, SETM1PID, 
			   SetDWORDval(kd),
			   SetDWORDval(kp),
			   SetDWORDval(ki),
			   SetDWORDval(qpps));
	}

	typedef struct {
		float p;
		float i;
		float d;
		uint32_t qpps;
	} TPID;

	TPID getM1PID() {
		writeN(2, portAddress, GETM1PID);
		uint8_t checksum = portAddress + GETM1PID;
		TPID result;

		result.d = getLongCont(checksum) / 65536.0;
		result.p = getLongCont(checksum) / 65536.0;
		result.i = getLongCont(checksum) / 65536.0;
		result.qpps = getLongCont(checksum);

		uint8_t responseChecksum = readByteWithTimeout();
		if ((checksum & 0x7F) != (responseChecksum & 0x7F)) {
			if (DEBUG) cout << "[getM1PID] Expected checksum of: " << hex << int(checksum) << ", but got:" << int(responseChecksum) << dec << endl;
			throw new TRoboClawException("[getM1PID] INVALID CHECKSUM");
		}

		return result;
	}

};

int main(int argc, char **argv) {
	cout << "Hello world\n";
	TRoboClaw roboClaw;

	try {
		string stringVal = roboClaw.getVersion();
		cout << "VERSION: " << stringVal << endl;
		float floatVal = roboClaw.getLogicBatteryLevel();
		cout << "LOGIC VOLTAGE: " << floatVal << endl;
		floatVal = roboClaw.getM1MaxCurrent();
		cout << "MAX M1 Motor Current: " << floatVal << endl;
		roboClaw.setM1PID(226.3538, 13.35421, 0, 2810);
		TRoboClaw::TPID pid = roboClaw.getM1PID();
		cout << "PID p: " << pid.p << ", i: " << pid.i << ", d: " << pid.d << ", q: " << pid.qpps << endl;
	} catch (TRoboClaw::TRoboClawException* e) {
		cout << "[main] EXCEPTION: " << e->what() << endl;
		return -1;
	}
}
