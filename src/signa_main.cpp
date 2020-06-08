#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/thread.hpp>
#include <signal.h>
#include <time.h>
#include <sys/stat.h>
#include <stdio.h>

#include "novatel/novatel.h"
#include "mavlink/tmtc_ladgnss.h"
//#include "linux_kbhit.h"

//#include "oem4.h"

using namespace novatel;
using namespace std;

Novatel my_gps;
Novatel my_modem;

int mode;
char start_time[50];
char LogDirName[50];

static int start = 0;
int SysMode_Save;
int SysMode_Log;
int Fault_Mode;

//
SerialPort SerialPort_GPS;
SerialPort SerialPort_Modem;
SerialPort SerialPort_FCC;

extern "C" bool InitUart(SerialPort* pSerialPort);
//

void BestUtmHandler(UtmPosition &pos, double &timestamp) {
//    std::cout << "[" << setprecision(2) << std::fixed << timestamp <<
//            setprecision(std::cout.precision()) <<  "] BestUtm: " <<
//            pos.header.gps_week << ":" << pos.header.gps_millisecs <<
//            "   Type: " << pos.position_type << " Pos: ("<<pos.easting<<
//            ","<<pos.northing<<")" << std::endl;
}

void BestVelHandler(Velocity &vel, double &timestamp) {
//    std::cout << "[" << setprecision(2) << std::fixed << timestamp <<
//        setprecision(std::cout.precision()) <<  "] BestVel: " <<
//        vel.header.gps_week << ":" << vel.header.gps_millisecs <<
//        "   Horiz: " << vel.horizontal_speed << " Course: "<<
//        vel.track_over_ground<<std::endl;
}

void RangeHandler(RangeMeasurements &ranges, double &timestamp) {

	int i;
	printf("[%ld]\r\n", ranges.header.gps_millisecs);
	int rn = ranges.number_of_observations;
	for (i = 0; i < rn / 2; i++) {
		if (ranges.range_data[2 * i].satellite_prn < 33) {
			printf("[%4i]", ranges.range_data[2 * i].satellite_prn);
		}
	}
	printf("\r\n");
	for (i = 0; i < rn / 2; i++) {
		if (ranges.range_data[2 * i].satellite_prn < 33) {
			printf("[%4i]",
					ranges.range_data[2 * i].channel_status.tracking_state);
		}
	}
	printf("\r\n");
	for (i = 0; i < rn / 2; i++) {
		if (ranges.range_data[2 * i].satellite_prn < 33) {
			printf("[%3.1f]", ranges.range_data[2 * i].carrier_to_noise);
		}
	}
	printf("\r\n");

}

void RawEphemerisHandler(RawEphemeris &raw_ephemeris, double &timestamp) {

}

void AlmanacHandler(Almanac &almanac, double &timestamp) {

}

void sighandler(int sig) {
	std::cout << "Exiting." << std::endl;

	my_gps.Disconnect(); // Close Novatel Port
	close(SerialPort_Modem.fd); // Close Mavlink Port
	close(SerialPort_FCC.fd);
	std::cerr << "MavLink : Serial port closed successfully. " << std::endl;

	cout << '\a';

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	exit(EXIT_SUCCESS);
}

int main(int argc, char **argv) {
	// catch ctrl-c
	signal(SIGINT, &sighandler);

	if (argc < 2) {
		std::cerr << "Failed : Input argument is not sufficient. " << std::endl;
		std::cerr << "[Command] ./signa 1 : Real-Time Mode" << std::endl;
		std::cerr << "[Command] ./signa 2 : Real-Time and Save Mode" << std::endl;
		std::cerr << "[Command] ./signa 3 : File Mode" << std::endl;
		return 0;
	}
	istringstream(argv[1]) >> mode;

	// Record start time for generating log file
	struct tm *t;
	time_t timer;
	timer = time(NULL);
	t = localtime(&timer);
	sprintf(start_time, "%02d%02d%02d", t->tm_year-100, t->tm_mon+1, t->tm_mday);

	// Serial port configuration
	strcpy(SerialPort_GPS.uartName, "/dev/ttyGPS"); // novatel
	strcpy(SerialPort_Modem.uartName, "/dev/ttyModem"); // modem
	strcpy(SerialPort_FCC.uartName, "/dev/ttyFCC"); // ubx

	SerialPort_GPS.baudRate = 115200; // novatel
	SerialPort_Modem.baudRate = 115200; // mavlink
	SerialPort_FCC.baudRate = 38400;

	// Run File Mode
//	if ((mode == 3)) {
//		my_gps.RunFileMode();
//		return 0;
//	}

	if ((mode == 2)) { // Save mode
		struct stat st = {0};
		if (stat("usb/log", &st) != -1){
			sprintf(LogDirName, "usb/log");
		} else {
			sprintf(LogDirName, "log");
		}
		SysMode_Save = 1; // on

		mode = 1; //
	}

	// Run Real-Time Mode
	if ((mode == 1)) {
		SysMode_Log = 1; // Log Mode on;

		// Open UBX
		if (!InitUart(&SerialPort_FCC)) {
			printf("Failed : Serial Port for Mavlink Failed to Open on %s\n", SerialPort_Modem.uartName);
			return 0;
		}
		//
		bool result = my_gps.Connect(SerialPort_GPS.uartName, SerialPort_GPS.baudRate, 1);

//		my_gps.FaultTest();

		if (result) {
			cout << "port 1 Successfully connected." << endl;
		} else {
			cout << "port 1 Failed to connect." << endl;
			return 0;
		}
//		printf("paas\n");

		result = my_gps.WarmStartReset();
//
//		if (result) {
//			cout << "port 1 Successfully reset." << endl;
//		} else {
//			cout << "port 1 Failed to reset." << endl;
//			return 0;
//		}
		printf("paas\n");

		// Open MavLink
		bool result2 = my_modem.Connect_mav(SerialPort_Modem.uartName, SerialPort_Modem.baudRate, 1);
		if (result2) {
			cout << "port 2 Successfully connected." << endl;
		} else {
			cout << "port 2 Failed to connect." << endl;
			return 0;
		}

	}

	if (start == 0){
		if ((mode == 1)) {
			my_gps.SendCommand("clockadjust enable", true);
			my_gps.SendCommand("dynamics foot", true);
			my_gps.SendCommand("ecutoff 5.0", true);
			my_gps.SendCommand("csmooth 100", true);
//
//			my_gps.ConfigureLogs("versionb once");
//			my_gps.ConfigureLogs("markposb once");
//			my_gps.ConfigureLogs("ionutcb once");

//			my_gps.ConfigureLogs("gpgga ontime 0.2");
			my_gps.ConfigureLogs("gphdtdualantenna ontime 0.2");
//			my_gps.ConfigureLogs("gpgsa ontime 0.5");
//			my_gps.ConfigureLogs("gpgsv ontime 0.5");
//			my_gps.ConfigureLogs("gprmc ontime 0.2");
//			my_gps.ConfigureLogs("gpvtg ontime 0.2");

			my_gps.ConfigureLogs("timeb ontime 0.1");
			my_gps.ConfigureLogs("bestposb ontime 0.2");
			my_gps.ConfigureLogs("bestvelb ontime 0.2");
			my_gps.ConfigureLogs("psrdopb onchanged");
			my_gps.ConfigureLogs("rangeb ontime 0.2");
			my_gps.ConfigureLogs("gpsephemb onchanged");
			my_gps.ConfigureLogs("gloephemerisb onchanged");
			start = 1;
		}
	}
	if (mode == 1){
		while (1) {
			boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
		}
	}
	return 0;
}
