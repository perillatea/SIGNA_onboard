#ifndef _SIGNA_H
#define _SIGNA_H

#include <stdio.h> // for printf
#include <string.h> // for memset
#include <stdint.h>
#include "config.h"
#include "oem4.h"

typedef struct {
	uint16_t   prn;      /* PRN nr of range measurements                     */
	uint16_t   filler;   /* reserved 2 bytes, not used                       */
	double     psr;      /* [m] Pseudo measurement [m]                       */
	float      psr_std;  /* [m] Pseudo measurement std deviation             */
	double     adr;      /* [cycles] Carrier phase (accumulated Doppler range)*/
	float      adr_std;  /* [cycles] Estimated carrier phase std deviation   */
	float      dopp;     /* [Hz] Doppler frequency                           */
	float      C_No;     /* [dB-Hz] Carrier to noise density ratio           */
	float      locktime; /* [s] Nr of continuos tracking (no cycle slip)     */
} SIGNA_RANGEB_CHANNEL;

// ------------- 구조 선언 -------------- //
typedef struct{
	// Measurement
	double 	mCode[kNumSatellite];
	double 	mPhase[kNumSatellite];
	float 		mDoppler[kNumSatellite];
	float 		mLockTime[kNumSatellite];
	float 		mCN0[kNumSatellite];
	uint32_t 	mFrequency[kNumSatellite]; // frequency
	float		mWaveLength[kNumSatellite];

	double 	mCodePrev[kNumSatellite];
	double 	mPhasePrev[kNumSatellite];
	float 	mLockTimePrev[kNumSatellite];

	double 	mCSC1[kNumSatellite]; // 100s carrier smoothed code measurement
	double 	mCSC1_prev[kNumSatellite]; // previous value
	uint8_t	mCSC1_count[kNumSatellite]; // continuous smoothing count
	double 	mCSC2[kNumSatellite]; // 30s carrier smoothed code measurement
	double 	mCSC2_prev[kNumSatellite]; // previous value
	uint8_t	mCSC2_count[kNumSatellite]; // continuous smoothing count

	double mCSC1_corrected[kNumSatellite];

	// Estimate
	double 	TimeTx_Raw[kNumSatellite];
	double 	TimeTx[kNumSatellite];
	double  TransitTime[kNumSatellite];
	double 	SvClockOffset[kNumSatellite];
	double 	SvClockDrift[kNumSatellite];
	double 	SvPosition[kNumSatellite][3]; // x,y,z in ECEF frame
	double 	SvELrad[kNumSatellite]; // Elevation, rad
	double 	SvAZrad[kNumSatellite]; // Azimuth, rad
	double 	SvPRC1[kNumSatellite];
	double 	SvRRC1[kNumSatellite];
	double 	SvPRC1Prev[kNumSatellite];
	double 	SvPRC2Prev[kNumSatellite];
	double 	SvPRC2[kNumSatellite];
	double 	SvRRC2[kNumSatellite];

	double	SvPRC[kNumSatellite]; // Test 용

	// Status
//	unsigned char sIsEphCurr[kNumSatellite];
//	unsigned char sInView[kNumSatellite];
//	uint16_t sContinuousEpoch[kNumSatellite];
//	uint16_t sEpochSinceStart;
//	double sEpochCurrent;
//	double sEpochPrevious;
//
//	unsigned char NumSv;
//	double RefPosition[3]; // meter
//	double RefLLA[3]; // lat, lon in radian, alt in meter
//	double UsrPosition[3];
//	double UsrLLA[3];
//	double UsrClockOffset;
//	double ISTB1;
} NavParameter;

typedef struct {
	// Status
	unsigned char sSvStatus[kNumSatellite];
	unsigned char sIsEphCurrOn[kNumSatellite];
	unsigned char sIsMeasurementOn[kNumSatellite];
	unsigned char sIsLowElevation[kNumSatellite];
	unsigned long sContinuousEpoch[kNumSatellite];
	unsigned long sEpochSinceStart;
	double sTimeCurrent;
	double sTimePrevious;

	unsigned char NumSv;
	unsigned char NumSvInUse;
	unsigned char NumSvGPS;
	unsigned char NumSvGLO;
	unsigned char NumConstellation;
	unsigned char sConstellationFlag;
	unsigned char PositioningMode;

	double HDOP;
	double VDOP;
	double PDOP;

	double RefPos_ECEF[3]; // meter
	double RefPos_LLA[3]; // lat, lon in radian, alt in meter
	double UsrPos_ECEF[3];
	double UsrPos_LLA[3];
	double UsrClockOffset;
	double ISTB1;
} NavSystem;

typedef struct {
	uint32_t prn;
	unsigned char flag;
	unsigned char status;
	unsigned char URA;
	unsigned char health;

	// Orbit Parameter
	double M0;
	double deltaN;
	double ecc;		// eccentricity
	double A;
	double i0;
	double idot;
	double cuc;
	double cus;
	double crc;
	double crs;
	double cic;
	double cis;
	double w; 			// omega
	double omegadot;	// omegadot
	double omega0;	// omega0
	double N;
	double TOE;
	uint32_t IODE1;
	uint32_t IODE2;

	// Clock Parameter
	unsigned wn; // week number;
	double Tgd;
	double af0;
	double af1;
	double af2;
	unsigned int TOC;
	uint32_t IODC;
} GPSEphData;

typedef struct {
	uint16_t prn;                       //!< Satellite PRN number
	uint16_t freqo;
	uint8_t sat_type;   //!< Ephemeris reference time [sec]
	uint8_t reserved;              //!< Subframe 1 data
	uint16_t e_week;              //!< Subframe 2 data
	uint32_t e_time;              //!< Subframe 3 data
	uint32_t t_offset;
	uint16_t Nt;                     //!< 32-bit cyclic redundancy check (CRC)
	uint8_t reserved2;
	uint8_t reserved3;
	uint32_t issue;
	uint32_t health;
	double posx;
	double posy;
	double posz;
	double velx;
	double vely;
	double velz;
	double accx;
	double accy;
	double accz;
	double taun;
	double ttaun;
	double gamma;
	uint32_t Tk;
	uint32_t P;
	uint32_t Ft;
	uint32_t age;
	uint32_t Flags;
	uint32_t CRC;
} GLOEphData;

typedef struct {
	GPSEphData GPS[kNumGPS];
	GLOEphData GLO[kNumGLO];
} NavEphData;

typedef struct {
	double 	SvPRC1[kNumSatellite];
	double 	SvRRC1[kNumSatellite];
	double 	SvPRC2[kNumSatellite];
	double 	SvRRC2[kNumSatellite];
	double	zcount;
	double	zcountsv[kNumSatellite];
	int		flag[kNumSatellite];
	uint32_t iod[kNumSatellite];
} GroundData;


///UBX

	typedef struct nav_posllh {
		int8_t header1;
		int8_t header2;
		int8_t classe;
		int8_t id;
		uint16_t length;
		uint32_t itow; /*gps millisecond time of week*/
		int32_t lon;
		int32_t lat;
		int32_t height;
		int32_t hMSL;
		uint32_t hAcc;
		uint32_t vAcc;
	} nav_posllh;

	typedef struct nav_status {
		int8_t header1;
		int8_t header2;
		int8_t classe;
		int8_t id;
		int16_t length;
		uint32_t itow;
		uint8_t gpsfix;
		int8_t flags;
		int8_t fixstat;
		int8_t flags2;
		uint32_t ttff;
		uint32_t msss;
	} nav_status;


	typedef struct nav_dop {
		int8_t header1;
		int8_t header2;
		int8_t classe;
		int8_t id;
		int16_t length;
		uint32_t itow;
		uint16_t gdop;
		uint16_t pdop;
		uint16_t tdop;
		uint16_t vdop;
		uint16_t hdop;
		uint16_t ndop;
		uint16_t edop;
	} nav_dop;

	typedef struct nav_sol {
		int8_t header1;
		int8_t header2;
		int8_t classe;
		int8_t id;
		int16_t length;
		uint32_t itow;
		uint32_t ftow;
		uint16_t week;
		uint8_t gpsfix;
		uint8_t flags;
		int32_t ecefx;
		int32_t ecefy;
		int32_t ecefz;
		uint32_t pAcc;
		int32_t ecefVX;
		int32_t ecefVY;
		int32_t ecefVZ;
		uint32_t sAcc;
		uint16_t pDOP;
		uint8_t reserved1;
		uint8_t numSV;
		uint32_t reserved2;
	} nav_sol;

	typedef struct nav_velned {
		int8_t header1;
		int8_t header2;
		int8_t classe;
		int8_t id;
		int16_t length;
		uint32_t itow;
		int32_t veln;
		int32_t vele;
		int32_t veld;
		uint32_t speed;
		uint32_t gspeed;
		int32_t heading;
		uint32_t sACC;
		uint32_t cACC;
	} nav_velned;
/* Space vehicle measurement for stacking */


// ------------- 함수 선언 -------------- //
void InitializeNavParam(NavSystem *NavSys, NavParameter *NavParam);
void UpdateNavMeasurement(NavSystem *NavSys, NavParameter *NavParam, OEM4_RANGEB_MSG *msg);
void LoadRefXYZ(NavSystem *NavSys);
void EstimateSvParam(NavSystem *NavSys, NavParameter *NavParam, NavEphData *NavEph);
void KeplerEquation(double *Ek, double Mk, double ecc);
void EstimateSvClock(NavParameter *NavParam, NavEphData *NavEph, unsigned char svidx, unsigned char FlagConstellation);
void EstimateSvOrbit(NavParameter *NavParam, NavEphData *NavEph, unsigned char svidx, unsigned char FlagConstellation);
void SvMotionEq_GLO(double *xvd, double *yvd, double *zvd, double *xp, double *yp, double *zp,
					double *xv, double *yv, double *zv, double *xa, double *ya, double *za);
void EstimateElAz(NavSystem *NavSys, NavParameter *NavParam, unsigned char svidx);
void EstimateCSC(NavParameter *NavParam, unsigned int svidx);
void EstimateCorr(NavSystem *NavSys, NavParameter *NavParam, unsigned char i);
void MonitorSvParam(NavSystem *NavSys, NavParameter *NavParam);
void PrintOutput(NavSystem *NavSys, NavParameter *NavParam);
void SaveCurrentSvInfo(NavSystem *NavSys, NavParameter *NavParam);
void ConvertECEF2LLA(double xyz[3],double xlla[3]);
void EstimateUsrPos(NavSystem *NavSys, NavParameter *NavParam);
void DetermineSvStatus(NavSystem *NavSys, NavParameter *NavParam, NavEphData *NavEph);
unsigned char GetConstFlag(unsigned char i);
void SendNMEAmsg(NavSystem *NavSys, NavParameter *NavParam);
void SendUBXmsg(NavSystem *NavSys);

#endif


