#include "mavlink/tmtc_ladgnss.h"
//#include "signa_proc.h"
#include "signa.h"

extern GroundData GndData;

void SleepMs(uint32_t timeoutMs);
void DoMsgCommon(mavlink_message_t* pMsg);
void DoMsgMeasurementBlock(mavlink_message_t* pMsg);
void DoMsgTransmitEnd(mavlink_message_t* pMsg);
//void RecvExample();
//void SendMsgToUsr(int svlist[kNumSatellite], RCVR *sRcvr, EXMSvBased *sExmSv,
//		EXMChannelBased *sExmChannel, MQMSystemBased *sMqmSys, SQMSystemBased *sSqmSys);
void SendMsgToUsr(NavSystem *NavSys, NavParameter *NavParam);

void mavlink_ParseBinary(unsigned char *message, unsigned int length){
	mavlink_message_t msg;
	mavlink_status_t status;
	int sv;

	int i;
	//    printf("%d \n", length);
	for (i = 0; i < length; i++){

		if (mavlink_parse_char(0, message[i], &msg, &status)){
			switch (msg.msgid) {

			// Received Msg Common
			case MAVLINK_MSG_ID_LADGNSS_COMMON:
				DoMsgCommon(&msg);
				//	          printf("Recv Example: 3 to open serial port \n");
				break;

				// Received Msg Mesurement Block
			case MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK:
				DoMsgMeasurementBlock(&msg);
				//	          printf("Recv Example: 4 to open serial port \n");
				break;

				// Received Msg Transmit End
			case MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END:
				DoMsgTransmitEnd(&msg);
				//	          printf("Recv Example: 5 to open serial port \n");
				break;
			default:
				printf("Error");
				break;
			}
		}
	}
}

void SleepMs(uint32_t timeoutMs) {
	timeoutMs += clock();
	while ((uint32_t) clock() < timeoutMs)
		continue;
}

void DoMsgCommon(mavlink_message_t* pMsg){
  mavlink_ladgnss_common_t msgRecv;
  mavlink_msg_ladgnss_common_decode(pMsg, &msgRecv);

  GndData.zcount = msgRecv.reference_point_height/10;  // need edit

//  printf("Lat: %d // Nmeas : %d\n", msgRecv.latitude, msgRecv.n_measurements);
}


void DoMsgMeasurementBlock(mavlink_message_t* pMsg){
	int sv;
  mavlink_ladgnss_measurement_block_t msgRecv;
  mavlink_msg_ladgnss_measurement_block_decode(pMsg, &msgRecv);

  sv = msgRecv.ranging_source_id-1; // need edit
  GndData.SvPRC1[sv] =  msgRecv.prc_100s/100.0;
  GndData.SvPRC2[sv] =  msgRecv.prc_30s/100.0;
  GndData.SvRRC1[sv] =  msgRecv.rrc_100s/100.0;
  GndData.SvRRC2[sv] =  msgRecv.rrc_30s/100.0;
  GndData.zcountsv[sv] = GndData.zcount;   // need edit
  GndData.iod[sv] = msgRecv.iod;   // need edit
  GndData.flag[sv] = 1;   // need edit

//  printf("[PRN %2i] [PRC100 %5.2f] [PRC30 %5.2f]\n",
//		  msgRecv.ranging_source_id, (double) msgRecv.prc_100s/100, (double) msgRecv.prc_30s/100);

}

void DoMsgTransmitEnd(mavlink_message_t* pMsg) {
	mavlink_ladgnss_transmit_end_t msgRecv;
	mavlink_msg_ladgnss_transmit_end_decode(pMsg, &msgRecv);

//	printf("Transmit Complete!\n");
}
//
//void RecvExample() {
//	uint32_t i;
//
//	SerialPort port;
//	mavlink_message_t msg;
//	mavlink_status_t status;
//
//	// 1. Init UART
//	strcpy(port.uartName, "COM9");
//	strcpy(port.uartName, COM9);
//	port.baudRate = 115200;
//
//	if (!InitUart(&port)) {
//		printf("Recv Example: Failed to open serial port %s\n", port.uartName);
//		return;
//	}
//	// 2. Read Mavlink
//
//	while (true) {
//		if (!RecvBytes(&port))
//			continue;
//
//		for (i = 0; i < port.nBytesRecv; i++) {
//
//			if (i >= sizeof(port.rxBuf))
//				break;
//
//			if (mavlink_parse_char(0, port.rxBuf[i], &msg, &status)) {
//				switch (msg.msgid) {
//
//				// Received Msg Common
//				case MAVLINK_MSG_ID_LADGNSS_COMMON:
//					DoMsgCommon(&msg);
//					printf("Recv Example: 3 to open serial port %s\n",
//							port.uartName);
//					break;
//
//					// Received Msg Mesurement Block
//				case MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK:
//					DoMsgMeasurementBlock(&msg);
//					printf("Recv Example: 4 to open serial port %s\n",
//							port.uartName);
//					break;
//
//					// Received Msg Transmit End
//				case MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END:
//					DoMsgTransmitEnd(&msg);
//					printf("Recv Example: 5 to open serial port %s\n",
//							port.uartName);
//					break;
//				default:
//					printf("Error");
//					break;
//				}
//			}
//		}
//
//	}
//	//*/
//	close(port.fd);
//}
void SendMsgToUsr(NavSystem *NavSys, NavParameter *NavParam){
	unsigned char i;
	extern SerialPort_Modem;
	uint8_t sv;
	int n = 0;
	int Nmeas = 0;

	mavlink_message_t msg;
	mavlink_ladgnss_common_t msgCommon;
	mavlink_ladgnss_measurement_block_t msgBlock[N_BLOCK];
	mavlink_ladgnss_transmit_end_t msgEnd;

	for (i=0; i<kNumSatellite; i++){
		if (NavSys->sIsMeasurementOn[i] == 1
				&& NavSys->sIsEphCurrOn[i] == 1
				&& NavParam->mCSC1_count[i] == kSmoothLen)
		Nmeas++;
	}
	printf("\n << Broadcast Message Status >> \t # of Satellites : %2i\n", Nmeas);
	printf("%c[1;30m",27); // 검정색
	printf("     [PRN] [ Corr1 ] [ RRC1 ] [ Corr2 ] [ RRC2 ] [ IOD ]  \n");
	printf("%c[0m",27);

	// Send Common
	msgCommon.n_measurements = Nmeas;
	msgCommon.latitude = 37;
	msgCommon.reference_point_height = NavSys->sTimeCurrent*10;

	mavlink_msg_ladgnss_common_encode(0, 0, &msg, &msgCommon);
	SendMavlink(&SerialPort_Modem, &msg);
//	SleepMs(10000);


	// Send Measurement Block
	for (i = 0; i < kNumSatellite; i++) {
		if (NavSys->sIsMeasurementOn[i] == 1
				&& NavSys->sIsEphCurrOn[i] == 1
				&& NavParam->mCSC1_count[i] == kSmoothLen){

			if (n >= N_BLOCK)
				continue;

			msgBlock[n].prc_100s 	= NavParam->SvPRC1[i] * 100;
			msgBlock[n].prc_30s 	= NavParam->SvPRC2[i] * 100;
			msgBlock[n].rrc_100s 	= NavParam->SvRRC1[i] * 100;
			msgBlock[n].rrc_30s 	= NavParam->SvRRC2[i] * 100;
//			msgBlock[n].iod 		= sRcvr->mIOD[sv];
			msgBlock[n].ranging_source_id = i + 1;

			mavlink_msg_ladgnss_measurement_block_encode(0, 0, &msg, &msgBlock[n]);
			SendMavlink(&SerialPort_Modem, &msg);

			// PrintOut
			if (i < kNumGPS){
				printf("%c[1;34m",27); // GPS 파란색
				printf("[GPS]");
				printf("%c[0m",27);
			} else if (i >= kNumGPS && i < kNumGPS + kNumGLO) {
				printf("%c[1;33m",27); // GLO 노란색
				printf("[GLO]");
				printf("%c[0m",27);
			}
			printf("  %2i    %5d   %5d      %5d   %5d      %3i  \n",
					msgBlock[n].ranging_source_id,
					msgBlock[n].prc_100s, msgBlock[n].rrc_100s,
					msgBlock[n].prc_30s, msgBlock[n].rrc_30s,
					msgBlock[n].iod);

			n++;
//			SleepMs(10000);
		}
	}

	// Send Transmit End
	mavlink_msg_ladgnss_transmit_end_encode(0, 0, &msg, &msgEnd);
	SendMavlink(&SerialPort_Modem, &msg);
} // function: SendCorrToUsr
