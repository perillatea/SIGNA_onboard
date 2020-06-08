/*
 * signa_core.c
 *
 *  Created on: 2019. 6. 3.
 *      Author: gnss
 */

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>

#include "oem4.h"
#include "config.h"
#include "navconst.h"
#include "signa.h"
#include "matops.h"
#include "mavlink/tmtc_ladgnss.h"

#include "nmea/nmea.h"

extern int SysMode_Save;
extern char start_time[50];
extern char LogDirName[50];

extern GroundData GndData;
extern float psrdop;
extern uint8_t numsat[2];
extern double bestPos[3];
extern double bestVel[2];
extern int timeutc[6];
extern double dualheading;
//extern char SaveFileName[50];
extern SerialPort SerialPort_FCC;

void SIGNA_core(OEM4_RANGEB_MSG *msg, NavEphData *NavEph){

	// Status
	static NavParameter NavParam;
	static NavSystem NavSys;
	static int init;
	unsigned char i;

	if (init == 0){
		LoadRefXYZ(&NavSys);
		init = 1;
		NavSys.sEpochSinceStart = 0;
		NavSys.UsrPos_ECEF[0] = bestPos[0];
		NavSys.UsrPos_ECEF[1] = bestPos[1];
		NavSys.UsrPos_ECEF[2] = bestPos[2];
//		NavSys.UsrPos_ECEF[0] = NavSys.RefPos_ECEF[0];
//		NavSys.UsrPos_ECEF[1] = NavSys.RefPos_ECEF[1];
//		NavSys.UsrPos_ECEF[2] = NavSys.RefPos_ECEF[2];
		NavSys.UsrClockOffset = 0;
		NavSys.ISTB1 = 0;
	}

	// 시스템 변수 초기화
	InitializeNavParam(&NavSys, &NavParam);

	// 측정치 업데이트
	UpdateNavMeasurement(&NavSys, &NavParam, msg);

	// IsEphCurrOn chk. DQM 추가 시 수정 예정
	for (i=0; i<kNumSatellite; i++){
		switch (GetConstFlag(i)){
		case FlagGPS :
			if (NavEph->GPS[i].prn == i+1)
				NavSys.sIsEphCurrOn[i] = 1;
			break;
		case FlagGLO :
			if (NavEph->GLO[i-kNumGPS].prn == i+1)
				NavSys.sIsEphCurrOn[i] = 1;
			break;
		}
	}
	// Continuous SV check
	for (i=0; i<kNumSatellite; i++){
		if (fabsf(NavParam.mLockTime[i] - NavParam.mLockTimePrev[i]) == 0.5){ // continue
			NavSys.sContinuousEpoch[i]++;
//			printf("[PRN] %2i [ContEpoch] %3i \n", i+1, NavParam.sContinuousEpoch[i]);
		} else {
			NavSys.sContinuousEpoch[i] = 1;;
			NavParam.mCSC1_count[i] = 1;
			NavParam.mCSC2_count[i] = 1;
		}
	}

	// Estimation
	EstimateSvParam(&NavSys, &NavParam, NavEph);

	// DetermineSvStatus
	DetermineSvStatus(&NavSys, &NavParam, NavEph);

	if (NavSys.NumSv < 6)
		NavSys.PositioningMode = StandAlone;
	else
		NavSys.PositioningMode = DGPS;

	switch (NavSys.PositioningMode) {
	case StandAlone:
		NavSys.UsrPos_ECEF[0] = bestPos[0];
		NavSys.UsrPos_ECEF[1] = bestPos[1];
		NavSys.UsrPos_ECEF[2] = bestPos[2];
		NavSys.HDOP = (double) psrdop;
		NavSys.NumSv = numsat[0];
		NavSys.NumSvInUse = numsat[1];

		break;
	case DGPS:
		EstimateUsrPos(&NavSys, &NavParam);
		NavSys.NumSvInUse = NavSys.NumSv;
		break;
	}
	ConvertECEF2LLA(NavSys.UsrPos_ECEF, NavSys.UsrPos_LLA);
//	printf("%i -----------------\n", NavSys.NumSv);
	// Integrity Monitoring
//	MonitorSvParam(&NavSys, &NavParam);

	// Print
//	PrintOutput(&NavSys, &NavParam);

	//	Broadcast Nav Data via NMEA message
	SendNMEAmsg(&NavSys, &NavParam);
//		SendUBXmsg(&NavSys);

	SaveCurrentSvInfo(&NavSys, &NavParam);

} // function: SIGNA_proc

void InitializeNavParam(NavSystem *NavSys, NavParameter *NavParam){

	int i;

	for (i=0; i<kNumSatellite; i++){
		// NavParam
		NavSys->sIsMeasurementOn[i] = 0;
		NavSys->sIsEphCurrOn[i] = 0;

		// NavParam
		NavParam->mCode[i] = 0;
		NavParam->mPhase[i] = 0;
		NavParam->mDoppler[i] = 0;
		NavParam->mCN0[i] = 0;
		NavParam->mLockTime[i] = 0;

		NavParam->TimeTx_Raw[i] = 0;
		NavParam->TimeTx[i] = 0;
		NavParam->SvClockOffset[i] = 0;
		NavParam->SvClockDrift[i] = 0;
		NavParam->SvPosition[i][0] = 0;
		NavParam->SvPosition[i][1] = 0;
		NavParam->SvPosition[i][2] = 0;
		NavParam->SvAZrad[i] = 0;
		NavParam->SvELrad[i] = 0;
		NavParam->SvPRC1[i] = 0;
		NavParam->SvPRC2[i] = 0;
		NavParam->SvRRC1[i] = 0;
		NavParam->SvRRC2[i] = 0;
	}
} // function: InitializeNavParam

void UpdateNavMeasurement(NavSystem * NavSys, NavParameter *NavParam, OEM4_RANGEB_MSG *msg) {
	int i;
	uint16_t idx;
	uint32_t number_of_obs = msg->obs;

	NavSys->sTimeCurrent = (double) msg->header.millisecs / 1000.0;
	NavSys->sEpochSinceStart++;

	for (i=0; i<number_of_obs; i++){
		// L1 데이터만 저장
		if (msg->ch[i].status.frequency != 0)
			continue;

		// Constellation 별 저장
		switch (msg->ch[i].status.satellite_system) {
		case FlagGPS: // GPS
			idx = msg->ch[i].prn - 1;

			// 현재 측정치 업로드
			NavParam->mCode[idx] 		= msg->ch[i].psr;
			NavParam->mPhase[idx] 		= msg->ch[i].adr;
			NavParam->mDoppler[idx] 	= msg->ch[i].dopp;
			NavParam->mCN0[idx] 		= msg->ch[i].C_No;
			NavParam->mLockTime[idx] 	= msg->ch[i].locktime;
			NavParam->mFrequency[idx]	= Freq_L1_GPS;
			NavParam->mWaveLength[idx]  = SPEED_OF_LIGHT/(double)NavParam->mFrequency[idx];

			NavSys->sIsMeasurementOn[idx] = 1;
			break;

		case FlagGLO: // GLONASS
			idx = msg->ch[i].prn - 6;

			NavParam->mCode[idx] 		= msg->ch[i].psr;
			NavParam->mPhase[idx] 		= msg->ch[i].adr;
			NavParam->mDoppler[idx] 	= msg->ch[i].dopp;
			NavParam->mCN0[idx] 		= msg->ch[i].C_No;
			NavParam->mLockTime[idx] 	= msg->ch[i].locktime;
			NavParam->mFrequency[idx]	= Freq_L1_GLO + (msg->ch[i].filler - 7) * FreqStepGLO;
			NavParam->mWaveLength[idx]  = SPEED_OF_LIGHT/(double)NavParam->mFrequency[idx];

			NavSys->sIsMeasurementOn[idx] = 1;
			break;
		}
	}
} // function: UpdateNavMeasurement

void LoadRefXYZ(NavSystem *NavSys){
	FILE *fp;
	fp = fopen("RefLLA.dbs","r");

	if (fp == 0){
		printf("ERR : Failed to load surveyed location file.\n");
		return;
	}

	// LLA 형식 Reference Station XYZ 읽기
	double latd, latm, lats, lond, lonm, lons;
	fscanf(fp, "%lf	%lf	%lf", &latd, &latm, &lats);
	fscanf(fp, "%lf %lf %lf", &lond, &lonm, &lons);
	fscanf(fp, "%lf", &NavSys->RefPos_LLA[2]);
	NavSys->RefPos_LLA[0] = (latd + latm/60.0 + lats/3600.0) * d2r; // Latitude
	NavSys->RefPos_LLA[1] = (lond + lonm/60.0 + lons/3600.0) * d2r; // Longitude

	// LLA to ECEF로 변경
	double rn;
	double sinlat, coslat, sinlon, coslon, alt;
	sinlat = sin(NavSys->RefPos_LLA[0]);
	coslat = cos(NavSys->RefPos_LLA[0]);
	sinlon = sin(NavSys->RefPos_LLA[1]);
	coslon = cos(NavSys->RefPos_LLA[1]);
	alt = NavSys->RefPos_LLA[2];


	rn = A_WGS84 / sqrt(1 - E2_Earth * sinlat * sinlat);
	NavSys->RefPos_ECEF[0] = (rn + alt) * coslat * coslon;
	NavSys->RefPos_ECEF[1] = (rn + alt) * coslat * sinlon;
	NavSys->RefPos_ECEF[2] = (rn * (1 - E2_Earth) + alt) * sinlat;
} // function: LoadRefXYZ

void EstimateSvParam(NavSystem *NavSys, NavParameter *NavParam, NavEphData *NavEph){

	int i;
	unsigned char NumSv = 0;

	for (i=0; i<kNumSatellite; i++){
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1){
			// Clock offset 적용 전 Transmission Time 추정
			NavParam->TimeTx_Raw[i] = NavSys->sTimeCurrent - NavParam->mCode[i]/SPEED_OF_LIGHT;

			// 위성 시계 오차, 궤도 추정
			switch (GetConstFlag(i)){
			case FlagGPS:
				EstimateSvClock(NavParam, NavEph, i, FlagGPS);

				NavParam->TimeTx[i] = NavParam->TimeTx_Raw[i] - NavParam->SvClockOffset[i];
				NavParam->TransitTime[i] = NavSys->sTimeCurrent - NavParam->TimeTx[i];

				EstimateSvOrbit(NavParam, NavEph, i, FlagGPS);
				break;

			case FlagGLO:
				EstimateSvClock(NavParam, NavEph, i, FlagGLO);

				NavParam->TimeTx[i] = NavParam->TimeTx_Raw[i] - NavParam->SvClockOffset[i];
				NavParam->TransitTime[i] = NavSys->sTimeCurrent - NavParam->TimeTx[i];

				EstimateSvOrbit(NavParam, NavEph, i, FlagGLO);
				break;
			}

			// 위성 El, Az 계산
			EstimateElAz(NavSys, NavParam, i);

			// Code-Carrier Smoothing
			EstimateCSC(NavParam, i);

//			// Corrected Psr 계산
//			NavParam->mCSC1_corrected[i] = NavParam->mCSC1[i] - GndData.SvPRC1[i];

//			EstimateCorr(NavParam, i);

			if (NavParam->SvELrad[i] > 5*d2r)
				NumSv++;
		}
	}
	NavSys->NumSv = NumSv;

} // function: EstimateSvParam

void KeplerEquation(double *Ek, double Mk, double ecc) {
	int max_iteration = 10;
	double diff_Ek = 10.0;
	double newEk;
	int i = 0;

	*Ek = Mk;
	while (diff_Ek > 1.0e-15 && i < max_iteration){
		newEk = Mk + (ecc * sin(*Ek));
		diff_Ek = fabs(newEk - *Ek);

		*Ek = newEk;
		i++;
	}
} // function: KeplerEquation

void EstimateSvClock(NavParameter *NavParam, NavEphData *NavEph, unsigned char i, unsigned char FlagConstellation) {

	double tk;

	if (FlagConstellation == FlagGPS){
		double n0, nk, Mk;
		double Ek, rel_t;
		double A, M0, deltaN, ecc, af0, af1, af2, TOC, Tgd;

		A 		= NavEph->GPS[i].A;
		M0 		= NavEph->GPS[i].M0;
		deltaN 	= NavEph->GPS[i].deltaN;
		ecc 	= NavEph->GPS[i].ecc;
		af0 	= NavEph->GPS[i].af0;
		af1 	= NavEph->GPS[i].af1;
		af2 	= NavEph->GPS[i].af2;
		TOC		= NavEph->GPS[i].TOC;
		Tgd 	= NavEph->GPS[i].Tgd;

		tk = NavParam->TimeTx_Raw[i] - TOC;

		// 상대성 효과에 의한 delay
		n0 = sqrt(Mu_WGS84 / (A * A * A));
		nk = n0 + deltaN;
		Mk = M0 + nk * tk;
		KeplerEquation(&Ek, Mk, ecc);
		rel_t = F_relative * ecc * sqrt(A) * sin(Ek);

		NavParam->SvClockOffset[i] = af0 + (af1 * tk) + (af2 * tk * tk) + rel_t + Tgd;
		NavParam->SvClockDrift[i]  = af1 + af2 * tk;
	}
	else if (FlagConstellation == FlagGLO){
		double e_time, taun, gamma;
		e_time = NavEph->GLO[i-kNumGPS].e_time;
		taun 	= NavEph->GLO[i-kNumGPS].taun;
		gamma  = NavEph->GLO[i-kNumGPS].gamma;

		tk = NavParam->TimeTx_Raw[i] - e_time;
		NavParam->SvClockOffset[i] = -taun + (gamma * tk);
	}
} // function: EstimateSvClock

void EstimateSvOrbit(NavParameter *NavParam, NavEphData *NavEph, unsigned char i, unsigned char FlagConstellation){

	if (FlagConstellation == FlagGPS){
		double tk, n0, nk, Mk;
		double Ek, nuk, phik;
		double A, ecc, w, M0, N, cic, cis, crc, crs, cuc, cus,
				deltaN, i0, idot, omega0, omegadot, TOE,
				uk, rk, ik, xo, yo, omegak;
		double xp_temp, yp_temp, zp_temp, tau;

		A 			= NavEph->GPS[i].A;
		ecc			= NavEph->GPS[i].ecc;
		M0 			= NavEph->GPS[i].M0;
		N 			= NavEph->GPS[i].N;
		cic 		= NavEph->GPS[i].cic;
		cis			= NavEph->GPS[i].cis;
		crc 		= NavEph->GPS[i].crc;
		crs 		= NavEph->GPS[i].crs;
		cuc 		= NavEph->GPS[i].cuc;
		cus 		= NavEph->GPS[i].cus;
		deltaN 	= NavEph->GPS[i].deltaN;
		i0 			= NavEph->GPS[i].i0;
		idot 		= NavEph->GPS[i].idot;
		omega0 	= NavEph->GPS[i].omega0;
		omegadot 	= NavEph->GPS[i].omegadot;
		w 			= NavEph->GPS[i].w;
		TOE			= NavEph->GPS[i].TOE;

		tk = NavParam->TimeTx[i] - TOE;

		n0 = sqrt(Mu_WGS84 / (A * A * A));
		nk = n0 + deltaN;
		Mk = M0 + nk * tk;
		KeplerEquation(&Ek, Mk, ecc);

		nuk = atan2(sqrt(1 - ecc * ecc) * sin(Ek) / (1 - ecc * cos(Ek)), (cos(Ek) - ecc) / (1 - ecc * cos(Ek)));
		phik = nuk + w;

		uk = cus * sin(2*phik) + cuc * cos(2*phik) + phik;
		rk = crs * sin(2*phik) + crc * cos(2*phik) + A * (1 - ecc * cos(Ek));
		ik = cis * sin(2*phik) + cic * cos(2*phik) + i0 + idot * tk;

		xo = rk * cos(uk);
		yo = rk * sin(uk);
		omegak = omega0 - (OmegaDot_WGS84 * TOE) + (omegadot - OmegaDot_WGS84) * tk;

		xp_temp = xo * cos(omegak) - yo * cos(ik) * sin(omegak);
		yp_temp = xo * sin(omegak) + yo * cos(ik) * cos(omegak);
		zp_temp = yo * sin(ik);

		// Earth Rotation
		tau = NavParam->TransitTime[i];
//		NavSys->sTimeCurrent - NavParam->TimeTx[i];

		NavParam->SvPosition[i][0] =  xp_temp * cos(tau * OmegaDot_WGS84) + yp_temp * sin(tau * OmegaDot_WGS84);
		NavParam->SvPosition[i][1] = -xp_temp * sin(tau * OmegaDot_WGS84) + yp_temp * cos(tau * OmegaDot_WGS84);
		NavParam->SvPosition[i][2] =  zp_temp;
	}
	else if (FlagConstellation == FlagGLO) {
		double tk, xp, yp, zp, xv, yv, zv, xa, ya, za; // Eph 변수 저장용
		double step; // step size
		double n, res; // iteration 횟수, 나머지
		int s;
		/* Runge Kutta 변수 */
		double xp1, xp2, xp3, xp4;
		double yp1, yp2, yp3, yp4;
		double zp1, zp2, zp3, zp4;
		double xv1, xv2, xv3, xv4;
		double yv1, yv2, yv3, yv4;
		double zv1, zv2, zv3, zv4;
		double xvd1, xvd2, xvd3, xvd4;
		double yvd1, yvd2, yvd3, yvd4;
		double zvd1, zvd2, zvd3, zvd4;
		double r, g, h, k, tau;
		double xp_temp, yp_temp, zp_temp, xv_temp, yv_temp, zv_temp;

		xp = NavEph->GLO[i-kNumGPS].posx;
		yp = NavEph->GLO[i-kNumGPS].posy;
		zp = NavEph->GLO[i-kNumGPS].posz;
		xv = NavEph->GLO[i-kNumGPS].velx;
		yv = NavEph->GLO[i-kNumGPS].vely;
		zv = NavEph->GLO[i-kNumGPS].velz;
		xa = NavEph->GLO[i-kNumGPS].accx;
		ya = NavEph->GLO[i-kNumGPS].accy;
		za = NavEph->GLO[i-kNumGPS].accz;

		/* Transmission time 계산 */
		tk = NavParam->TimeTx[i] - NavEph->GLO[i-kNumGPS].e_time;

		step = 60.0 * tk/fabs(tk); // tk의 부호 반영

		/* iteration 횟수 계산 */
		n = floor(fabs(tk / step));
		res = fmod(tk, step);
		if (res != 0.0)
			n++;

		// Runge Kutta
		xp_temp = xp; yp_temp = yp; zp_temp = zp;
		xv_temp = xv; yv_temp = yv; zv_temp = zv;

		for (s=0; s<n; s++){
			// 나머지가 0이 아닌 경우, 마지막의 step size를 나머지의 크기로 변경
			if (res != 0.0 && s == n-1)
				step = res;

			// step 1
			xp1 = xp_temp;
			yp1 = yp_temp;
			zp1 = zp_temp;
			xv1 = xv_temp;
			yv1 = yv_temp;
			zv1 = zv_temp;
			SvMotionEq_GLO(&xvd1, &yvd1, &zvd1, &xp1, &yp1, &zp1, &xv1, &yv1, &zv1, &xa, &ya, &za);

			// step 2
			xp2 = xp_temp + xv1 * step/2.0;
			yp2 = yp_temp + yv1 * step/2.0;
			zp2 = zp_temp + zv1 * step/2.0;
			xv2 = xv_temp + xvd1 * step/2.0;
			yv2 = yv_temp + yvd1 * step/2.0;
			zv2 = zv_temp + zvd1 * step/2.0;
			SvMotionEq_GLO(&xvd2, &yvd2, &zvd2, &xp2, &yp2, &zp2, &xv2, &yv2, &zv2, &xa, &ya, &za);

			// step 3
			xp3 = xp_temp + xv2 * step/2.0;
			yp3 = yp_temp + yv2 * step/2.0;
			zp3 = zp_temp + zv2 * step/2.0;
			xv3 = xv_temp + xvd2 * step/2.0;
			yv3 = yv_temp + yvd2 * step/2.0;
			zv3 = zv_temp + zvd2 * step/2.0;
			SvMotionEq_GLO(&xvd3, &yvd3, &zvd3, &xp3, &yp3, &zp3, &xv3, &yv3, &zv3, &xa, &ya, &za);

			// step 4
			xp4 = xp_temp + xv3 * step;
			yp4 = yp_temp + yv3 * step;
			zp4 = zp_temp + zv3 * step;
			xv4 = xv_temp + xvd3 * step;
			yv4 = yv_temp + yvd3 * step;
			zv4 = zv_temp + zvd3 * step;
			SvMotionEq_GLO(&xvd4, &yvd4, &zvd4, &xp4, &yp4, &zp4, &xv4, &yv4, &zv4, &xa, &ya, &za);

			xp_temp = xp_temp + (xv1 + 2*xv2 + 2*xv3 + xv4) * step/6.0;
			yp_temp = yp_temp + (yv1 + 2*yv2 + 2*yv3 + yv4) * step/6.0;
			zp_temp = zp_temp + (zv1 + 2*zv2 + 2*zv3 + zv4) * step/6.0;
			xv_temp = xv_temp + (xvd1 + 2*xvd2 + 2*xvd3 + xvd4) * step/6.0;
			yv_temp = yv_temp + (yvd1 + 2*yvd2 + 2*yvd3 + yvd4) * step/6.0;
			zv_temp = zv_temp + (zvd1 + 2*zvd2 + 2*zvd3 + zvd4) * step/6.0;
		}
		// Earth Rotation
		tau = NavParam->TransitTime[i];
//				NavParam->sTimeCurrent - NavParam->TimeTx[i];
		xp_temp =  xp_temp * cos(tau * OmegaDot_PZ90) + yp_temp * sin(tau * OmegaDot_PZ90);
		yp_temp = -xp_temp * sin(tau * OmegaDot_PZ90) + yp_temp * cos(tau * OmegaDot_PZ90);

		//	Datum 변환 (PZ-90 to WGS84)
		NavParam->SvPosition[i][0] = xp_temp - 0.36;
		NavParam->SvPosition[i][1] = yp_temp + 0.08;
		NavParam->SvPosition[i][2] = zp_temp + 0.18;
	}
} // function: EstimateSvOrbit

void SvMotionEq_GLO(double *xvd, double *yvd, double *zvd, double *xp, double *yp, double *zp,
					double *xv, double *yv, double *zv, double *xa, double *ya, double *za){
	double r, g, h, k;

	r = sqrt((*xp)*(*xp) + (*yp)*(*yp) + (*zp)*(*zp));
	g = -Mu_PZ90 / (r*r*r);
	h = J2_PZ90 * 1.5 * (A_PZ90 * A_PZ90 / (r*r));
	k = 5 * (*zp)*(*zp) / (r*r);

	*xvd = g*(*xp)*(1-h*(k-1)) + (*xa) + OmegaDot_PZ90*OmegaDot_PZ90*(*xp) + 2*OmegaDot_PZ90*(*yv);
	*yvd = g*(*yp)*(1-h*(k-1)) + (*ya) + OmegaDot_PZ90*OmegaDot_PZ90*(*yp) - 2*OmegaDot_PZ90*(*xv);
	*zvd = g*(*zp)*(1-h*(k-3)) + (*za);
} // function: SvMotionEq_GLO

void EstimateElAz(NavSystem *NavSys, NavParameter *NavParam, unsigned char i){
	double dx, dy, dz, slat, slon, r;
	double enu[3];

	// XYZ to ENU 변환
	dx = NavParam->SvPosition[i][0] - NavSys->RefPos_ECEF[0];
	dy = NavParam->SvPosition[i][1] - NavSys->RefPos_ECEF[1];
	dz = NavParam->SvPosition[i][2] - NavSys->RefPos_ECEF[2];

	slat = PI/2.0 - NavSys->RefPos_LLA[0];
	slon = PI/2.0 + NavSys->RefPos_LLA[1];

	enu[0] = cos(slon)*dx + sin(slon)*dy;
	enu[1] = -cos(slat)*sin(slon)*dx + cos(slat)*cos(slon)*dy + sin(slat)*dz;
	enu[2] = sin(slat)*sin(slon)*dx - sin(slat)*cos(slon)*dy + cos(slat)*dz;

	r = sqrt(enu[0]*enu[0] + enu[1]*enu[1]);
	NavParam->SvELrad[i] = atan2(enu[2], r);
	NavParam->SvAZrad[i] = atan2(enu[0], enu[1]);
} // function: EstimateElAz

void EstimateCSC(NavParameter *NavParam, unsigned int i){
	double Code, Phase;
	uint8_t n1, n2;
	double temp;
	double CSC1, CSC2;
	double PhasePrev, CSC1Prev, CSC2Prev;
	float WaveLength;

	Code = NavParam->mCode[i];
	Phase = NavParam->mPhase[i];
	PhasePrev = NavParam->mPhasePrev[i];
	CSC1Prev = NavParam->mCSC1_prev[i];
	CSC2Prev = NavParam->mCSC2_prev[i];
	WaveLength = NavParam->mWaveLength[i];

	n1 = NavParam->mCSC1_count[i];
	n2 = NavParam->mCSC2_count[i];

	// 100s Smoothing
	if (n1 == 1){
		CSC1 = Code;
		n1++;
	} else {
		n1 = MIN(n1, kSmoothLen);
		temp = (Phase - PhasePrev)*WaveLength;
		CSC1 = Code/n1 + (CSC1Prev - temp) * (n1-1)/n1;
		if (n1 < kSmoothLen)
			n1++;
	}
	NavParam->mCSC1_count[i] = n1;
	NavParam->mCSC1[i] = CSC1;

	// 30s Smoothing
	if (n2 == 1){
		CSC2 = Code;
		n2++;
	} else {
		n2 = MIN(n2, kSmoothLen2);
		temp = (Phase - PhasePrev)*WaveLength;
		CSC2 = Code/n2 + (CSC2Prev - temp) * (n2-1)/n2;
		if (n2 < kSmoothLen2)
			n2++;
	}
	NavParam->mCSC2_count[i] = n2;
	NavParam->mCSC2[i] = CSC2;
} // function: EstimateCSC

void EstimateCorr(NavSystem *NavSys, NavParameter *NavParam, unsigned char i){
	double CSC1, CSC2, dx, dy, dz, r, ClockOffset;

	CSC1 			= NavParam->mCSC1[i];
	CSC2 			= NavParam->mCSC2[i];
	dx 				= NavParam->SvPosition[i][0] - NavSys->RefPos_ECEF[0];
	dy				= NavParam->SvPosition[i][1] - NavSys->RefPos_ECEF[1];
	dz 				= NavParam->SvPosition[i][2] - NavSys->RefPos_ECEF[2];
	ClockOffset 	= NavParam->SvClockOffset[i];

	r  = sqrt(dx*dx + dy*dy + dz*dz);

	if (NavParam->mCSC1_count[i] == kSmoothLen)
		NavParam->SvPRC1[i] = CSC1 - r + ClockOffset*SPEED_OF_LIGHT;

	if (NavParam->mCSC2_count[i] == kSmoothLen2)
		NavParam->SvPRC2[i] = CSC2 - r + ClockOffset*SPEED_OF_LIGHT;

	NavParam->SvRRC1[i] = (NavParam->SvPRC1[i] - NavParam->SvPRC1Prev[i]) / 2.0;
	NavParam->SvRRC2[i] = (NavParam->SvPRC2[i] - NavParam->SvPRC2Prev[i]) / 2.0;

	NavParam->SvPRC[i] = NavParam->mCode[i] - r + ClockOffset*SPEED_OF_LIGHT;

} // function: EstimateCorr

void PrintOutput(NavSystem *NavSys, NavParameter *NavParam){

	int i;

	// Header
	printf("\033[H\033[J");
	printf("%c[1;95m",27); // 마젠타
	printf("\n\t SIGNA v1.0 \t\t\t\t\t\t");
	printf("Epochs Since Start : %d(%.1f s) \n\n", NavSys->sEpochSinceStart, (float) NavSys->sEpochSinceStart/2.0);

	printf("** Position[ECEF] %8.2f %8.2f %8.2f   [Error] %8.2f %8.2f %8.2f\n",
			NavSys->UsrPos_ECEF[0], NavSys->UsrPos_ECEF[1], NavSys->UsrPos_ECEF[2],
			NavSys->UsrPos_ECEF[0]-NavSys->RefPos_ECEF[0],
			NavSys->UsrPos_ECEF[1]-NavSys->RefPos_ECEF[1],
			NavSys->UsrPos_ECEF[2]-NavSys->RefPos_ECEF[2]);
	printf("++ Position[ECEF] %8.2f %8.2f %8.2f   [Error] %8.2f %8.2f %8.2f\n",
			bestPos[0], bestPos[1], bestPos[2],
			bestPos[0]-NavSys->RefPos_ECEF[0],
			bestPos[1]-NavSys->RefPos_ECEF[1],
			bestPos[2]-NavSys->RefPos_ECEF[2]);

	printf("%c[1;30m",27); // 검정색
	printf("     [PRN] [SCount] [   El   ] [ PRCorg ] [ PRC100 ] [ RRC100 ] [ PRC30 ] [ RRC30 ] [    SvStatus    ] \n");
	printf("%c[0m",27);

	// Content
	for (i = 0; i < kNumSatellite; i++) {
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1){
//		if (NavSys->sSvStatus[i] == 1) {
			switch (GetConstFlag(i)) {
			case FlagGPS:
				printf("%c[1;34m",27); // GPS 파란색
				printf("[GPS]");
				printf("%c[0m",27);
				break;
			case FlagGLO:
				printf("%c[1;33m",27); // GLO 노란색
				printf("[GLO]");
				printf("%c[0m",27);
				break;
			}

			printf("  %2i    %3i       %5.2f   %7.2f      %6.2f     %6.2f    %6.2f    %6.2f",
				i+1,NavParam->mCSC1_count[i],
				NavParam->SvELrad[i]*r2d,NavParam->SvPRC[i],
				GndData.SvPRC1[i], GndData.SvRRC1[i],
				GndData.SvPRC2[i], GndData.SvRRC2[i]);
			if (NavSys->sSvStatus[i] == 1)
				printf("\t     Good");

			if (NavSys->sIsEphCurrOn[i] != 1)
				printf("\tInvalid Ephemeris");

			if (NavSys->sTimeCurrent - GndData.zcountsv[i] >= 5)
				printf("\tCorr. Timeout(%5.1f)", NavSys->sTimeCurrent - GndData.zcountsv[i]);

			if (NavParam->SvELrad[i] <= (MaskAngle * d2r))
				printf("\tLow Elevation");

			printf("\n");
		}
	}
} // function: PrintOutput

void SaveCurrentSvInfo(NavSystem *NavSys, NavParameter *NavParam){
	uint8_t i;

	NavSys->sTimePrevious = NavSys->sTimeCurrent;

	for (i=0;i<kNumSatellite;i++){
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1){
			NavParam->mCodePrev[i] = NavParam->mCode[i];
			NavParam->mPhasePrev[i] = NavParam->mPhase[i];
			NavParam->mLockTimePrev[i] = NavParam->mLockTime[i];

			NavParam->mCSC1_prev[i] = NavParam->mCSC1[i];
			NavParam->mCSC2_prev[i] = NavParam->mCSC2[i];
			NavParam->SvPRC1Prev[i] = NavParam->SvPRC1[i];
			NavParam->SvPRC2Prev[i] = NavParam->SvPRC2[i];
		} else {
			NavParam->mCodePrev[i] = NavParam->mCode[i];
			NavParam->mPhasePrev[i] = NavParam->mPhase[i];
			NavParam->mLockTimePrev[i] = NavParam->mLockTime[i];

			NavParam->mCSC1_prev[i] = NavParam->mCSC1[i];
			NavParam->mCSC2_prev[i] = NavParam->mCSC2[i];
			NavParam->SvPRC1Prev[i] = NavParam->SvPRC1[i];
			NavParam->SvPRC2Prev[i] = NavParam->SvPRC2[i];
		}
	}
} // function: SaveCurrentParam

void ConvertECEF2LLA(double xyz[3],double xlla[3]){
	int i;
	double rhosqrd, rho, templat, tempalt,
			rhoerror, zerror, slat, clat, q, r_n, drdl, aa, bb, cc, dd, invdet;

	xlla[1] = atan2(xyz[1], xyz[0])*r2d;


	rhosqrd = xyz[0]*xyz[0] + xyz[1]*xyz[1];
	rho = sqrt(rhosqrd);
	templat = atan2(xyz[2], rho);
	tempalt = sqrt(rhosqrd + xyz[2]*xyz[2]) - A_WGS84;
	rhoerror = 1000.0;
	zerror   = 1000.0;

	while ((fabs(rhoerror) > 1e-6) || (fabs(zerror) > 1e-6))
	{
		slat = sin(templat);
		clat = cos(templat);
		q = 1 - E2_Earth*slat*slat;
		r_n = A_WGS84/sqrt(q);
		drdl = r_n*E2_Earth*slat*clat/q;

		rhoerror = (r_n + tempalt)*clat - rho;
		zerror   = (r_n*(1 - E2_Earth) + tempalt)*slat - xyz[2];

		aa = drdl*clat - (r_n + tempalt)*slat;
		bb = clat;
		cc = (1 - E2_Earth)*(drdl*slat + r_n*clat);
		dd = slat;

		invdet = 1.0/(aa*dd - bb*cc);
		templat = templat - invdet*(+dd*rhoerror -bb*zerror);
		tempalt = tempalt - invdet*(-cc*rhoerror +aa*zerror);
	}

	xlla[0] = templat*r2d;
	xlla[2] = tempalt;
} // function: ConvertECEF2LLA

void EstimateUsrPos(NavSystem *NavSys, NavParameter *NavParam) {
	unsigned char NumSv, NumConstellation;
	double drho;
	int i, j, k;
	unsigned char SvRow = 1;
	double el;
	double Norm_dX = 100;

	double sigma;
	double sigTotal;
	double Re = 6378.1363;
	double xair = 0.2;
	double tauair = 100;
	double vair = 0;
	double dI = 0.004;
	double hI = 350;
	double sigPr, Fpp, sigIono, sigAir, sigTropo, sigTot;

	dmat G = ZERODMAT;
	dmat sigmaMatrix = ZERODMAT;
	dmat W = ZERODMAT;
	dmat transG = ZERODMAT;
	dmat transGxW = ZERODMAT;
	dmat transGxWxG = ZERODMAT;
	dmat invtransGxWxG = ZERODMAT;
	dmat H = ZERODMAT;
	dvec residual = ZERODVEC;
	dvec dX = ZERODVEC;

	NumSv = NavSys->NumSv;
	NumConstellation = NavSys->NumConstellation;

	initdmat(&G, 1, NumSv, 1, 3+NumConstellation); //G
	initdmat(&sigmaMatrix, 1, NumSv, 1, NumSv); //sigmaMatrix
	initdmat(&W, 1, NumSv, 1, NumSv);
	initdmat(&transG, 1, 3+NumConstellation, 1, NumSv);
	initdmat(&transGxW, 1, 3+NumConstellation, 1, NumSv);
	initdmat(&transGxWxG, 1, 3+NumConstellation, 1, 3+NumConstellation);
	initdmat(&invtransGxWxG, 1, 3+NumConstellation, 1, 3+NumConstellation);
	initdmat(&H, 1, 3+NumConstellation, 1, NumSv);
	initdvec(&residual, 1, NumSv);
	initdvec(&dX, 1, 3+NumConstellation);
	zerodmat(&sigmaMatrix);

	// S matrix
	for (i = 0; i < kNumSatellite; i++) {
//		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1 &&
//				GndData.SvPRC1[i] != 0.0 && NavParam->SvELrad[i] > (5*d2r)){
		if (NavSys->sSvStatus[i] == 1) {
			el = NavParam->SvELrad[i];

			sigPr = 0.16 + 1.07 * exp(-el / 15.5);
			Fpp = 1 / sqrt(1 - ((Re * cos(el) / (Re + hI))*(Re * cos(el) / (Re + hI))));
			sigIono = dI * (xair + 2 * tauair * vair) * Fpp;
			sigAir = 0.16 + 1.07 * exp(-el / 15.5);
			sigTropo = 0;
			sigTotal= sigPr*sigPr + sigIono*sigIono + sigAir*sigAir + sigTropo*sigTropo;

			setdmat(&sigmaMatrix, SvRow, SvRow, sigTotal);	//x
//			setdmat(&sigmaMatrix, SvRow, SvRow, 1);	//x

			SvRow++;
		}
	}
	// Get W by inverting sigmaMatrix
	dmatinv(&sigmaMatrix, &W);

	double dx, dy, dz, r;

	// Calculate Position
	while(Norm_dX > 0.001) {
		SvRow = 1;
		for (i = 0; i < kNumSatellite; i++) {
			if (NavSys->sSvStatus[i] == 1) {

				dx = NavParam->SvPosition[i][0] - NavSys->UsrPos_ECEF[0];
				dy = NavParam->SvPosition[i][1] - NavSys->UsrPos_ECEF[1];
				dz = NavParam->SvPosition[i][2] - NavSys->UsrPos_ECEF[2];
				r = sqrt(dx*dx + dy*dy + dz*dz);

				// update G matrix
				setdmat(&G, SvRow, 1, -1*dx/r);	//x
				setdmat(&G, SvRow, 2, -1*dy/r);	//x
				setdmat(&G, SvRow, 3, -1*dz/r);	//x

				switch (NavSys->NumConstellation) {
				case 1:
					setdmat(&G, SvRow, 4, 1);
//					drho = NavParam->mCode[i] - GndData.SvPRC1[i] -
//							(r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
//									+ NavSys->UsrClockOffset);
					drho = NavParam->mCSC1[i] - GndData.SvPRC1[i] -
							(r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
									+ NavSys->UsrClockOffset);
					break;
				case 2:
					if (GetConstFlag(i) == FlagGPS) {
						setdmat(&G, SvRow, 4, 1);
						setdmat(&G, SvRow, 5, 0);
					} else if (GetConstFlag(i) == FlagGLO) {
						setdmat(&G, SvRow, 4, 1);
						setdmat(&G, SvRow, 5, 1);
					}
//					drho = NavParam->mCode[i] - GndData.SvPRC1[i] -
//							(r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
//									+ NavSys->UsrClockOffset + NavSys->ISTB1);
					drho = NavParam->mCSC1[i] - GndData.SvPRC1[i] -
							(r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
									+ NavSys->UsrClockOffset + NavSys->ISTB1);
					break;
				}
				setdvec(&residual, SvRow, drho);	//range
				SvRow++;
			}
		}
		// Positioning
		transdmat(&G, &transG);
		dmatxdmat(&transG, &W, &transGxW);
		dmatxdmat(&transGxW, &G, &transGxWxG);
		dmatinv(&transGxWxG, &invtransGxWxG);
		dmatxdmat(&invtransGxWxG, &transGxW, &H);	//H=inv(G'WG)G'W
		dmatxdvec(&H,&residual,&dX); // After residual calculation

		NavSys->UsrPos_ECEF[0] = NavSys->UsrPos_ECEF[0] + dX.vec[1];
		NavSys->UsrPos_ECEF[1] = NavSys->UsrPos_ECEF[1] + dX.vec[2];
		NavSys->UsrPos_ECEF[2] = NavSys->UsrPos_ECEF[2] + dX.vec[3];
		NavSys->UsrClockOffset = NavSys->UsrClockOffset + dX.vec[4];

		if (NavSys->sConstellationFlag == FlagGPSGLO)
			NavSys->ISTB1 = NavSys->ISTB1 + dX.vec[5];

		Norm_dX = sqrt(dX.vec[1]*dX.vec[1] + dX.vec[2]*dX.vec[2] + dX.vec[3]*dX.vec[3]);
	}
	// DOP
	dmat transGxG = ZERODMAT;
	dmat invtransGxG = ZERODMAT;
	initdmat(&transGxG, 1, 3+NumConstellation, 1, 3+NumConstellation);
	initdmat(&invtransGxG, 1, 3+NumConstellation, 1, 3+NumConstellation);

	dmatxdmat(&transG, &G, &transGxG);
	dmatinv(&transGxG, &invtransGxG);

	NavSys->HDOP = sqrt(invtransGxG.mat[1][1] + invtransGxG.mat[2][2]);
	NavSys->VDOP = sqrt(invtransGxG.mat[3][3]);
	NavSys->PDOP = sqrt(invtransGxG.mat[1][1] + invtransGxG.mat[2][2] + invtransGxG.mat[3][3]);

	freedmat(&transGxG);
	freedmat(&invtransGxG);

	freedmat(&G);
	freedmat(&sigmaMatrix);
	freedmat(&W);
	freedmat(&transG);
	freedmat(&transGxW);
	freedmat(&transGxWxG);
	freedmat(&invtransGxWxG);
	freedmat(&H);
	freedvec(&residual);
	freedvec(&dX);
} // function: EstimateUsrPos

void DetermineSvStatus(NavSystem *NavSys, NavParameter *NavParam, NavEphData *NavEph){
	unsigned int i;

	NavSys->NumSv = 0;
	NavSys->NumSvGPS = 0;
	NavSys->NumSvGLO = 0;
	for (i = 0; i < kNumSatellite; i++) {
//		printf("%d \n", GndData.zcountsv[i]);

		if (NavSys->sIsMeasurementOn[i] == 1
				&& NavSys->sIsEphCurrOn[i] == 1
				&& (NavSys->sTimeCurrent - GndData.zcountsv[i]) < 5
				&& NavParam->SvELrad[i] > (MaskAngle * d2r)) {
			switch (GetConstFlag(i)) {
			case FlagGPS:
//				if ((NavEph->GPS[i].IODE1 == GndData.iod[i])
//						&& (ConstellationMode == 0 || ConstellationMode == 2)) {
				if (ConstellationMode == 0 || ConstellationMode == 2) {
					NavSys->NumSv++;
					NavSys->NumSvGPS++;
					NavSys->sSvStatus[i] = 1; // Good
				}
				break;
			case FlagGLO:
//				if ((NavEph->GLO[i-kNumGPS].e_time == GndData.iod[i]) // check
//						&& (ConstellationMode == 1 || ConstellationMode == 2)) {
				if (ConstellationMode == 1 || ConstellationMode == 2) {
					NavSys->NumSv++;
					NavSys->NumSvGLO++;
					NavSys->sSvStatus[i] = 1; // Good
				}
				break;
			} // switch
		} // if
		else
			NavSys->sSvStatus[i] = 0; // Bad
	} // for

	// Set Current Constellation Status
	if (NavSys->NumSvGLO == 0){
		NavSys->sConstellationFlag = 0;
		NavSys->NumConstellation = 1;
	} else if (NavSys->NumSvGPS == 0) {
		NavSys->sConstellationFlag = 1;
		NavSys->NumConstellation = 1;
	} else if ((NavSys->NumSvGPS > 0) && (NavSys->NumSvGLO > 0)) {
		NavSys->sConstellationFlag = 2;
		NavSys->NumConstellation = 2;
	}
} // function: DetermineSvStatus

unsigned char GetConstFlag(unsigned char i){
	// Return constellation flag
	// 0 : GPS
	// 1 : GLO
	// 9 : Not identified
	unsigned char prn;

	prn = i+1;
	if (prn >= PRN_GPS_min && prn <= PRN_GPS_max)
		return 0;
	else if (prn >= PRN_GLO_min && prn <= PRN_GLO_max)
		return 1;
	else
		return 9;

} // function: GetConstFlag

void SendNMEAmsg(NavSystem *NavSys, NavParameter *NavParam){

	nmeaINFO info;
	char buff[2048];
	int gen_sz;

	nmea_zero_INFO(&info);

	info.utc.year = timeutc[0];
	info.utc.mon = timeutc[1];
	info.utc.day = timeutc[2];
	info.utc.hour = timeutc[3];
	info.utc.min = timeutc[4];
	info.utc.sec = floor(timeutc[5]/1000);
	info.utc.hsec = floor((timeutc[5] - floor(timeutc[5]/1000)*1000)/10);

	switch (NavSys->PositioningMode) {
	case StandAlone:
		info.sig = 1;
		break;
	case DGPS:
		info.sig = 2;
		break;
	}
	info.satinfo.inuse = NavSys->NumSvInUse;
	info.satinfo.inview = NavSys->NumSv;
	info.fix = 3;

	info.PDOP = NavSys->PDOP;
	info.HDOP = NavSys->HDOP;
	info.VDOP = NavSys->VDOP;

	if (NavSys->UsrPos_LLA[0] > 0)
		info.lat = floor(NavSys->UsrPos_LLA[0])*100.0 + (NavSys->UsrPos_LLA[0]-floor(NavSys->UsrPos_LLA[0]))*60.0;
	else
		info.lat = -floor(-NavSys->UsrPos_LLA[0])*100.0 + (-NavSys->UsrPos_LLA[0]-floor(NavSys->UsrPos_LLA[0]))*60.0;
	if (NavSys->UsrPos_LLA[1] < 0)
		info.lon = floor(NavSys->UsrPos_LLA[1])*100.0 + (NavSys->UsrPos_LLA[1]-floor(NavSys->UsrPos_LLA[1]))*60.0;
	else
		info.lon = -floor(-NavSys->UsrPos_LLA[1])*100.0 + (-NavSys->UsrPos_LLA[1]-floor(NavSys->UsrPos_LLA[1]))*60.0;
	info.elv = NavSys->UsrPos_LLA[3];

	info.speed = bestVel[0] * NMEA_TUS_MS;
	info.direction = bestVel[1];

	info.heading = dualheading;
	// Generate NMEA message using struct. info
	gen_sz = nmea_generate( &buff[0], 2048, &info, GPGGA | GPRMC | GPVTG | GPHDT);
//	gen_sz = nmea_generate( &buff[0], 2048, &info, GPGGA | GPRMC | GPHDT | GPVTG | GPGSV);
	buff[gen_sz] = 0;

	// Send NMEA msg to FCC via serial comm.
	write(SerialPort_FCC.fd, buff, gen_sz);
	printf("%f ccc\n", dualheading);
	printf("\nFeeding NMEA to FCC, TX MSG below --- No.Sv %2i \n%s\n", NavSys->NumSv, &buff[0]);

	if (SysMode_Save) {
		FILE *NMEALog;
		char NMEALogName[50];

		sprintf(NMEALogName, "%s/NMEALog_%s.txt", LogDirName, start_time);
		NMEALog = fopen(NMEALogName, "ab");

		fprintf(NMEALog, "%s\n",&buff[0]);
		fclose (NMEALog);
	}

} // function: SendNMEAmsg

void SendUBXmsg(NavSystem *NavSys) {
	//JS

int cflag = 1;
int Numvalid = 8;
double xlla[3], xecef[3];

ConvertECEF2LLA(NavSys->UsrPos_ECEF,NavSys->UsrPos_LLA);
xlla[0] = NavSys->UsrPos_LLA[0];
xlla[1] = NavSys->UsrPos_LLA[1];
xlla[2] = NavSys->UsrPos_LLA[2];
xecef[0] = NavSys->UsrPos_ECEF[0];
xecef[1] = NavSys->UsrPos_ECEF[1];
xecef[2] = NavSys->UsrPos_ECEF[2];
double current_time;
current_time = NavSys->sTimeCurrent;

printf("%f %f %f\n", xlla[0], xlla[1], xlla[2]);

#define put_int8_t(buf, wire_offset, b) *(int8_t *)&buf[wire_offset] = b
#define put_uint8_t(buf, wire_offset, b) *(uint8_t *)&buf[wire_offset] = b
#define put_uint32_t(buf, wire_offset, b) *(uint32_t *)&buf[wire_offset] = b
#define put_int32_t(buf, wire_offset, b)  *(int32_t *)&buf[wire_offset] = b
#define put_int16_t(buf, wire_offset, b)  *(int16_t *)&buf[wire_offset] = b
#define put_uint16_t(buf, wire_offset, b)  *(uint16_t *)&buf[wire_offset] = b

	extern SerialPort SerialPort_FCC;

	printf("\n +----------------------------------+");
	printf("\n |      JS: UBLOX data write        |");
	printf("\n +----------------------------------+");

//	printf("Novatel :: %f %f %f\n",
//			best_xyz.x_position, best_xyz.x_position, best_xyz.x_position);

	/*------------------------------- Write data to serial port -----------------------------*/

	/*------------------------------- posllh -----------------------------*/

	nav_posllh msgCommon1;
	// Send Common
	msgCommon1.header1 = 181;
	msgCommon1.header2 = 98;
	msgCommon1.classe = 1;
	msgCommon1.id = 2;
	msgCommon1.length = 28;
	//Do not change up to this line.

	msgCommon1.itow = current_time*1000;	//ms
	msgCommon1.lon = xlla[1]/1e-7;	//deg. Do not remove scale parameter (1e-7)
	msgCommon1.lat = xlla[0]/1e-7;	//deg Do not remove scale parameter (1e-7)
	msgCommon1.height = xlla[2]*1000;	//mm
	msgCommon1.hMSL = 0*1000;	//mm
	msgCommon1.hAcc = sqrt(1)*1000;	//mm
	msgCommon1.vAcc = 1*1000;	//mm

	char buf1[36] = { 0 };//4 (header, ID) + 28 byte (length) for posecef 2 (length_write) +2 (checksum)=36

	//header
	put_int8_t(buf1, 0, msgCommon1.header1);
	put_int8_t(buf1, 1, msgCommon1.header2);

	//class-id
	put_int8_t(buf1, 2, msgCommon1.classe);
	put_int8_t(buf1, 3, msgCommon1.id);

	//length
	put_uint16_t(buf1, 4, msgCommon1.length);

	//payload
	put_uint32_t(buf1, 6, msgCommon1.itow);
	put_int32_t(buf1, 10, msgCommon1.lon);
	put_int32_t(buf1, 14, msgCommon1.lat);
	put_int32_t(buf1, 18, msgCommon1.height);
	put_int32_t(buf1, 22, msgCommon1.hMSL);
	put_uint32_t(buf1, 26, msgCommon1.hAcc);
	put_uint32_t(buf1, 30, msgCommon1.vAcc);

	//Checksum
	unsigned int cka = 0, ckb = 0;
	int i;
	int len = 32;	//28+4
	for (i = 2; i < len + 2; i++) {
		cka += buf1[i];
		ckb += cka;
	}

	put_uint8_t(buf1, 34, cka);
	put_uint8_t(buf1, 35, ckb);

	// ------------------------------------------STATUS-------------------------------------------

	nav_status msgCommon2;

	// Send Common
	msgCommon2.header1 = 181;
	msgCommon2.header2 = 98;
	msgCommon2.classe = 1;
	msgCommon2.id = 3;
	msgCommon2.length = 16;
	//Do not change up to this line.


	msgCommon2.itow = current_time*1000;	//ms
	msgCommon2.gpsfix = 3;//0x00: no fix, 2: 2D fix, 3: 3-D fix, for other, see description-JS
	msgCommon2.flags = 13;	//see Description. binary value of 13 is 1101 --> This means numbers are valid, dgps is not working.
	msgCommon2.fixstat = 1;	//see Description. DGPS related. If dgps needed to be used, modify.
	msgCommon2.flags2 = 0;	//see Description. //power save mode
	//msgCommon2.ttff = 222.992*1000;	//see Description. //time to first fix in milli sec
	//msgCommon2.msss = 1000;	//see Description. //milliseconds since startup/reset//
	msgCommon2.ttff = 0;	//see Description. //time to first fix in milli sec
	msgCommon2.msss = 0;	//see Description. //milliseconds since startup/reset//

	char buf2[24] = { 0 };	//8+length=24

	//header

	put_int8_t(buf2, 0, msgCommon2.header1);
	put_int8_t(buf2, 1, msgCommon2.header2);

	//class-id
	put_int8_t(buf2, 2, msgCommon2.classe);
	put_int8_t(buf2, 3, msgCommon2.id);

	//length
	put_uint16_t(buf2, 4, msgCommon2.length);
	//put_uint16_t(buf2, 5, msgCommon2.length);

	//payload
	put_uint32_t(buf2, 6, msgCommon2.itow);
	put_uint8_t(buf2, 10, msgCommon2.gpsfix);
	put_int8_t(buf2, 11, msgCommon2.flags);
	put_int8_t(buf2, 12, msgCommon2.fixstat);
	put_int8_t(buf2, 13, msgCommon2.flags2);
	put_uint32_t(buf2, 14, msgCommon2.ttff);
	put_uint32_t(buf2, 18, msgCommon2.msss);

	//Checksum
	cka = 0, ckb = 0;
	i = 0;
	len = 20;	//16+4
	//for (i = 2; i < len - 2; i++) {
	for (i = 2; i < len + 2; i++) {
		cka += buf2[i];
		ckb += cka;
	}

	put_uint8_t(buf2, 22, cka);
	put_uint8_t(buf2, 23, ckb);

	// ------------------------------------------DOP-------------------------------------------

	nav_dop msgCommon3;

	// Send Common
	msgCommon3.header1 = 181;
	msgCommon3.header2 = 98;
	msgCommon3.classe = 1;
	msgCommon3.id = 4;
	msgCommon3.length = 18;
	//Do not change up to this line.

	msgCommon3.itow = current_time*1000;	//ms
	msgCommon3.gdop = 1 / 0.01; //Do not remove scale parameter 0.01
	msgCommon3.pdop = NavSys->PDOP / 0.01;
	msgCommon3.tdop = 1 / 0.01;
	msgCommon3.vdop = NavSys->VDOP / 0.01;
	msgCommon3.hdop = 3 / 0.01;
	msgCommon3.ndop = 1 / 0.01;
	msgCommon3.edop = 1 / 0.01;

	char buf3[26] = { 0 };	//8+length

	//header

	put_int8_t(buf3, 0, msgCommon3.header1);
	put_int8_t(buf3, 1, msgCommon3.header2);

	//class-id
	put_int8_t(buf3, 2, msgCommon3.classe);
	put_int8_t(buf3, 3, msgCommon3.id);

	//length
	put_uint16_t(buf3, 4, msgCommon3.length);
	//put_uint16_t(buf3, 5, msgCommon3.length);

	//payload
	put_uint32_t(buf3, 6, msgCommon3.itow);
	put_uint16_t(buf3, 10, msgCommon3.gdop);
	put_uint16_t(buf3, 12, msgCommon3.pdop);
	put_uint16_t(buf3, 14, msgCommon3.tdop);
	put_uint16_t(buf3, 16, msgCommon3.vdop);
	put_uint16_t(buf3, 18, msgCommon3.hdop);
	put_uint16_t(buf3, 20, msgCommon3.ndop);
	put_uint16_t(buf3, 22, msgCommon3.edop);

	//Checksum
	cka = 0, ckb = 0;
	i = 0;
	len = 22; //length+4
	//for (i = 2; i < len - 2; i++) {
	for (i = 2; i < len + 2; i++) {
		cka += buf3[i];
		ckb += cka;
	}

	put_uint8_t(buf3, 24, cka);
	put_uint8_t(buf3, 25, ckb);

	// ------------------------------------------SOL------------------------------------------------------

	nav_sol msgCommon4;

	// Send Common
	msgCommon4.header1 = 181;
	msgCommon4.header2 = 98;
	msgCommon4.classe = 1;
	msgCommon4.id = 6;
	msgCommon4.length = 52;
	//Do not change up to this line.

	msgCommon4.itow = current_time*1000;	//ms
	msgCommon4.ftow = 10;	//ns
//	msgCommon4.week = 1963;	//gps week
	msgCommon4.week = 0;	//gps week
	msgCommon4.gpsfix = 3;//0x00: no fix, 2: 2D fix, 3: 3-D fix, for other, see description-JS
	msgCommon4.flags = 13;//see Description. binary value of 13 is 1101 --> This means numbers are valid, dgps is not working. -> changed to 1111
	msgCommon4.ecefx = xecef[0]*100;	//cm
	msgCommon4.ecefy = xecef[1]*100;	//cm
	msgCommon4.ecefz = xecef[2]*100;	//cm
	msgCommon4.pAcc = 50;	//cm
//	msgCommon4.pAcc = 0;	//cm
	msgCommon4.ecefVX = 0*100;	//cm/s
	msgCommon4.ecefVY = 0*100;	//cm/s
	msgCommon4.ecefVZ = 0*100;	//cm/s
	//msgCommon4.sAcc = 10;	//cm/s
	msgCommon4.sAcc = 1;	//cm/s
	msgCommon4.pDOP = cflag / 0.01;
	// cflag ::

	//
	msgCommon4.reserved1 = 0;
	msgCommon4.numSV = Numvalid;
	//msgCommon4.numSV = 5;
	msgCommon4.reserved2 = 0;

	char buf4[60] = { 0 };	//length+8

	//header
	put_int8_t(buf4, 0, msgCommon4.header1);
	put_int8_t(buf4, 1, msgCommon4.header2);

	//class-id
	put_int8_t(buf4, 2, msgCommon4.classe);
	put_int8_t(buf4, 3, msgCommon4.id);

	//length
	put_uint16_t(buf4, 4, msgCommon4.length);

	//payload
	put_uint32_t(buf4, 6, msgCommon4.itow);
	put_int32_t(buf4, 10, msgCommon4.ftow);
	put_int16_t(buf4, 14, msgCommon4.week);
	put_uint8_t(buf4, 16, msgCommon4.gpsfix);
	put_int8_t(buf4, 17, msgCommon4.flags);
	put_int32_t(buf4, 18, msgCommon4.ecefx);
	put_int32_t(buf4, 22, msgCommon4.ecefy);
	put_int32_t(buf4, 26, msgCommon4.ecefz);
	put_uint32_t(buf4, 30, msgCommon4.pAcc);
	put_int32_t(buf4, 34, msgCommon4.ecefVX);
	put_int32_t(buf4, 38, msgCommon4.ecefVY);
	put_int32_t(buf4, 42, msgCommon4.ecefVZ);
	put_uint32_t(buf4, 46, msgCommon4.sAcc);
	put_uint16_t(buf4, 50, msgCommon4.pDOP);
	put_uint8_t(buf4, 52, msgCommon4.reserved1);
	put_uint8_t(buf4, 53, msgCommon4.numSV);
	put_uint32_t(buf4, 54, msgCommon4.reserved2);

	//Checksum
	cka = 0, ckb = 0;
	i = 0;
	len = 56;	//length+4
	//for (i = 2; i < len - 2; i++) {
	for (i = 2; i < len + 2; i++) {
		cka += buf4[i];
		ckb += cka;
	}

	put_uint8_t(buf4, 58, cka);
	put_uint8_t(buf4, 59, ckb);

	// ------------------------------------------VELNED-------------------------------------------

	nav_velned msgCommon5;

	// Send Common
	msgCommon5.header1 = 181;
	msgCommon5.header2 = 98;
	msgCommon5.classe = 01;
	msgCommon5.id = 18;	//12 : JS: Ublox applies in this way. 12 should appear in HEX format.
	msgCommon5.length = 36;
	//Do not change up to this line.

	msgCommon5.itow = current_time*1000;	//ms
	msgCommon5.veln = 0*100;	//cm/s
	msgCommon5.vele = 0*100;	//cm/s
	msgCommon5.veld = 0*100;	//cm/s
	msgCommon5.speed = 0;	//cm/s
	msgCommon5.gspeed = 0;	//cm/s
//	msgCommon5.gspeed = 5;	//cm/s

	//msgCommon5.heading = 50 / 1e-5;	//deg. Do not remove 1e-5 (scaling parameter)
	msgCommon5.heading = 90 / 1e-5;	//deg. Do not remove 1e-5 (scaling parameter)
//	msgCommon5.sACC = 150;	//cm/s
//	msgCommon5.cACC = 40 / 1e-5;	//deg.  Do not remove 1e-5 (scaling parameter)
	msgCommon5.sACC = 1;	//cm/s
		msgCommon5.cACC = 0 / 1e-5;	//deg.  Do not remove 1e-5 (scaling parameter)

	char buf5[44] = { 0 };	//8+length

	//header
	put_int8_t(buf5, 0, msgCommon5.header1);
	put_int8_t(buf5, 1, msgCommon5.header2);

	//class-id
	put_int8_t(buf5, 2, msgCommon5.classe);
	put_int8_t(buf5, 3, msgCommon5.id);

	//length
	put_uint16_t(buf5, 4, msgCommon5.length);

	//payload
	put_uint32_t(buf5, 6, msgCommon5.itow);
	put_int32_t(buf5, 10, msgCommon5.veln);
	put_int32_t(buf5, 14, msgCommon5.vele);
	put_int32_t(buf5, 18, msgCommon5.veld);
	put_uint32_t(buf5, 22, msgCommon5.speed);
	put_uint32_t(buf5, 26, msgCommon5.gspeed);
	put_int32_t(buf5, 30, msgCommon5.heading);
	put_uint32_t(buf5, 34, msgCommon5.sACC);
	put_uint32_t(buf5, 38, msgCommon5.cACC);

	//Checksum
	cka = 0, ckb = 0;
	i = 0;
	len = 40;	//4+length
	//for (i = 2; i < len - 2; i++) {
	for (i = 2; i < len + 2; i++) {
		cka += buf5[i];
		ckb += cka;
	}

	put_uint8_t(buf5, 42, cka);
	put_uint8_t(buf5, 43, ckb);

	//---------------------------------------------------------------------------------
//	void SleepMs(uint32_t timeoutMs) {
//		timeoutMs += clock();
//		while ((uint32_t) clock() < timeoutMs)
//			continue;
//	}

//	SleepMs(500000);

	//bytes_written = write(fd, write_buffer, sizeof(write_buffer));/* use write() to send data to port*/

	//Mandatory for gps to be fixed.
	write(SerialPort_FCC.fd, buf1, sizeof(buf1));		//POSLLH
	write(SerialPort_FCC.fd, buf2, sizeof(buf2));		//STATUS
	write(SerialPort_FCC.fd, buf5, sizeof(buf5));		//VELNED

	//Optional for gps to be fixed. But informations are needed.
	write(SerialPort_FCC.fd, buf3, sizeof(buf3));		//DOP
	write(SerialPort_FCC.fd, buf4, sizeof(buf4));		//SOL

	//	close(fd);/* Close the Serial port */
	printf("JS: UBX Message Sent! \n\n");




}
