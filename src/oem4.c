/*******************************************************************************
 * IMT - Integrity Monitor Testbed 
 * Author:  Per-Ludvig Normark <ludde@relgyro.stanford.edu> 
 *
 * This is the OEM4 Rx binary format. This file contains fkn that repacks the 
 * OEM4 fromat into the IMT format.
 *
 * Change log:
 * 2001-01-19  Version 0.1 of this file
 * 2002-01-15  Added call to IMT_MSG_update_length() in repacking fkn's to 
 *             calculate the proper length of IMT msgs in repacking fkn's
 ******************************************************************************/
#include "oem4.h"
#include "imt_msg.h"
#include <stdio.h> // for printf
#include <string.h> // for memset

/* CRC c-code taken form OEM4 manual 2 */
#define CRC32_POLYNOMIAL   0xEDB88320L
/* --------------------------------------------------------------------------
Calculate a CRC value to be used by CRC calculation functions. 
-------------------------------------------------------------------------- */
unsigned long CRC32Value(int i)
{
  int j;
  unsigned long ulCRC;
  ulCRC = i;
  for ( j = 8 ; j > 0; j-- )
    {
      if ( ulCRC & 1 )
	ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
      else
	ulCRC >>= 1;
    }
  return ulCRC;
}
/* --------------------------------------------------------------------------
Calculates the CRC-32 of a block of data all at once
-------------------------------------------------------------------------- */
unsigned long CalculateBlockCRC32(unsigned long ulCount,
				  unsigned char *ucBuffer ) /* Data block */
{
  unsigned long ulTemp1;
  unsigned long ulTemp2;
  unsigned long ulCRC = 0;
  while ( ulCount-- != 0 )
    {
      ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
      ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
      ulCRC = ulTemp1 ^ ulTemp2;
    }
  return( ulCRC );
}


/* scan for synch bytes OBS! bytes_to_check = bytes_in_buf - HEADER_SIZE 
 * Actually exactly the same as for OEM3 */
int OEM4_found_header(unsigned char *buf,int start_buf, 
		      int bytes_to_check,int *start_msg)
{
  int i;
  /* check everything for a header (minimum size) */
  for(i=start_buf; i <= start_buf+bytes_to_check; i++){


    /* check for synch bytes 0xaa , 0x44 , 0x11  */
    if((buf[i] == (unsigned char)OEM4_SOP1) && 
       (buf[i+1] == (unsigned char)OEM4_SOP2) && 
       (buf[i+2] == (unsigned char)OEM4_SOP3)){
      *start_msg = i; /* where header starts */
      return 1;
    } 	
    /* step on to the next byte..*/
  }
  
  /* check for a possible broken header start */
  *start_msg = -1;
  return -1;
}

/* get the size of the data including the header. Must have a "full"
 * header (bytes_in_buf >= HEADER_SIZE*/
unsigned long OEM4_get_msg_size(unsigned char *buf)
{
  unsigned char header_size;
  unsigned short msg_size;
  unsigned long size;

  /* offset 8, within millenium header */
  header_size = *(unsigned char *)&buf[3];

  memcpy(&msg_size,&(buf[8]),2);
//  printf("oem4/msg size is %i\r\n",msg_size);

  size = header_size + msg_size + 4; //add the CRC size of 4 bytes
  return size;
}

/* get the size of the data including the header. Must have a "full"
 * header (bytes_in_buf >= HEADER_SIZE*/
unsigned long OEM4_get_header_size(unsigned char *buf)
{
  unsigned char header_size;

  header_size = *(unsigned char *)&buf[3];
//  printf("header size is %i\r\n",header_size);

  return header_size;
}


/* calculates the checksum and returns 1 if ok -1 if failed */
int OEM4_checksum_ok(unsigned char *msg_buf,unsigned long msg_size)
{
  unsigned long CRC_calculated;
  /* calculate the CRC, should be 0 */
  CRC_calculated = CalculateBlockCRC32(msg_size, msg_buf);
  return CRC_calculated;
}

/* simple for now */
void OEM4_rx_status(unsigned long status)
{
  printf("Antenna Open %i (0) shorted %i (0)\n",(int)(status & 0x20)>>5,
	 (int)(status & 0x40)>>6);
  printf("Com1 overflow %i (0), CPU overload %i (0)\n",(int)(status & 0x100)>>8, 
	 (int)(status % 0x80)>>7);
  printf("almanac valid:%i (0) pos_valid:%i (0)",(int)(status & 0x40000) >> 18,
	 (int)(status & 0x80000) >> 19);
}

int OEM4_repack_RANGE_to_IMT_RANGE(OEM4_RANGEB_MSG *range, unsigned char *rgeb,int rx_nr){
  int obs;
  int i;
  int j;
  unsigned long freq;
  IMT_RANGE_MSG *imt_range;
  IMT_RANGE_CHANNEL *imt_range_ch;
  OEM4_RANGEB_CHANNEL *oem4_range_ch;
  double gps_sec;
  unsigned long channel_nr;
  imt_range = (IMT_RANGE_MSG *)rgeb;



  /* zero out a IMT_MSG size*/
  memset(imt_range,0,sizeof(IMT_MSG));

  /* fill in header fields */
  imt_range->header.type = IMT_RANGE;
  imt_range->header.rx_nr = (unsigned char)rx_nr;
  imt_range->header.rx_type = OEM4_RX;
  imt_range->header.status = 0;
  /* only indicate that the message should be synchronized if the time mode 
   * is in fine or finesteering */
  if((range->header.gps_stat == GPSTIME_FINE) || (range->header.gps_stat == GPSTIME_FINESTEERING))
    imt_range->header.time_status = GPS_TIME;
  else
    imt_range->header.time_status = NO_TIME;
  imt_range->header.gps_week = (unsigned long)(range->header.gps_week % 1024);
  gps_sec = (double)(range->header.millisecs);
  imt_range->header.gps_sec = gps_sec / 1000.0;
  imt_range->header.pc_sec = range->header.pc_sec2;
  imt_range->header.pc_usec = range->header.pc_usec2;

  /* Time messages, GPS-ION_2001 Ludde */
  /* copy the time into the IMT msg,  put it on offset [980] (safe, 12 ch) 
   * t1 : rgeb[980]   idle time 1023-36 = 987
   * t2 : rgeb[988]   decode time
   * t3 : rgeb[996]   synch_time added in rx_synch
   * t4 : rgeb[1004]  process_time added in mrcc.c */
  //memcpy(&rgeb[980], range, 16);

  /* copy each channel into the IMT range channel */
  obs = range->obs;

  j = 0;
  for(i=0; i<obs; i++){
    /* copy the ptr's for easier writing (and compiler optim) */
    imt_range_ch  = &(imt_range->l1_data[j]);
    oem4_range_ch = &(range->ch[i]);    
    freq = (unsigned long)oem4_range_ch->status.frequency;
    if(freq == 0){ /* L1==0 (L2==1) Only L1 measurements, copy that */
      /* A quick and dirty test to verify innovationflag problems - ludde 8-oct-2001 */
      //if(oem4_range_ch->locktime > 60.0) {
	/* copy all variables into the channel */
	imt_range_ch->prn       = (unsigned long)oem4_range_ch->prn;
	imt_range_ch->psr       = oem4_range_ch->psr;
	imt_range_ch->adr       = oem4_range_ch->adr;
	imt_range_ch->dop       = oem4_range_ch->dopp;
	imt_range_ch->sno       = oem4_range_ch->C_No;
	imt_range_ch->locktime  = oem4_range_ch->locktime;
	/* get OEM4 channel */
	channel_nr = (unsigned long)oem4_range_ch->status.sv_channel_nr;
	imt_range_ch->ch = channel_nr;
	imt_range_ch->phase_lock = (unsigned char)oem4_range_ch->status.phase_lock;
	imt_range_ch->code_lock  = (unsigned char)oem4_range_ch->status.code_lock;
	imt_range_ch->parity_known = (unsigned char)oem4_range_ch->status.parity_known;
	j++;
	//}
	}
  }
  imt_range->l1_obs = j;

  /* add correct IMT byte length to the message header */
  IMT_MSG_update_length((IMT_MSG *)imt_range);

  return 1;
}

void OEM4_print_RANGE(OEM4_RANGEB_MSG *range){
  int obs;
  int i;
  obs = range->obs;
  printf("Observables[%i]\n",obs);
  for(i=0; i<obs; i++){
    printf("Prn[%i]  freq[%s] lock[%i] C/No[%f]\n",range->ch[i].prn,
	   (range->ch[i].status.frequency==0)?"L1":"L2",
	   range->ch[i].status.tracking_state,
	   range->ch[i].C_No);
    printf("         psr[%0.9g]  locktime[%0.8f]\n",
	   range->ch[i].psr, range->ch[i].locktime);
  }
}

void IMT_print_RANGE(IMT_RANGE_MSG *range){
  int obs;
  int i;
  obs = range->l1_obs;
  printf("Observables[%i]\n",obs);
  for(i=0; i<obs; i++){
    printf("Prn[%i]  C/No[%f]\n",(int)range->l1_data[i].prn,
	   range->l1_data[i].sno);
    printf("         psr[%0.9g]   locktime[%0.8f]\n",
	   range->l1_data[i].psr, range->l1_data[i].locktime);
  }
}

void print_OEM4_RANGE_and_IMT_RANGE(OEM4_RANGEB_MSG *range, 
				    IMT_RANGE_MSG *imt_range)
{
  int obs;
  int i;
  obs = range->obs;
  printf("Observables OEM4:[%i] IMT:[%i]\n",obs,(int)imt_range->l1_obs);
  printf("OEM4: GPStime=%u\n",(unsigned int)range->header.millisecs);
  printf("IMT:  GPStime=%0.9g\n",imt_range->header.gps_sec);
  printf("OEM4: GPSweek=%u\n",(unsigned int)range->header.gps_week);
  printf("IMT:  GPSweek=%u\n",(unsigned int)imt_range->header.gps_week);
  for(i=0; i<obs; i++){
    printf("OEM4: Prn[%i]  C/No[%f] freq[%s] lock[%i]\n",
	   range->ch[i].prn,
	   range->ch[i].C_No,
	   (range->ch[i].status.frequency==0)?"L1":"L2",
	   range->ch[i].status.tracking_state);
    printf("         psr[%0.9g]   locktime[%0.8f]\n",
	   range->ch[i].psr, 
	   range->ch[i].locktime);

    /* IMT message */
    printf("OEM4: Prn[%i]  C/No[%f] freq[l1] lock[%i]\n",
	   (int)imt_range->l1_data[i].prn,
	   imt_range->l1_data[i].sno,
	   imt_range->l1_data[i].code_lock);
    printf("         psr[%0.9g]  locktime[%0.8f]\n",
	   imt_range->l1_data[i].psr, 
	   imt_range->l1_data[i].locktime);
  }
}

                                       
#define  PI  3.1415926535898

/* which_sv must be a nr from 0..almanac->msgs since the OEM4 outputs all
 * known almanacs in one message, but the IMT expects one message per prn */
int OEM4_repack_ALMANAC_to_IMT_ALMANAC(OEM4_ALMANACB_MSG *almanac, 
				       unsigned char *almb, unsigned long which_sv,int rx_nr)
{
  IMT_ALMANAC_MSG  *imt_almanac;
  OEM4_ALMANACB_SV *almanac_sv;
  double gps_sec;

  imt_almanac = (IMT_ALMANAC_MSG *)almb;

  /* zero out a IMT_MSG size*/
  memset(imt_almanac,0,sizeof(IMT_MSG));

  /* fill in header fields */
  imt_almanac->header.type = IMT_ALMANAC;
  imt_almanac->header.rx_nr = (unsigned char)rx_nr;
  imt_almanac->header.rx_type = OEM4_RX;
  imt_almanac->header.status = 0;
  /* The almanac message uses SV_TIME */
  if((almanac->header.gps_stat == GPSTIME_SATTIME))
    imt_almanac->header.time_status = SV_TIME;
  else
    imt_almanac->header.time_status = NO_TIME;
  imt_almanac->header.gps_week = (unsigned long)(almanac->header.gps_week % 1024);
  gps_sec = (double)(almanac->header.millisecs);
  imt_almanac->header.gps_sec = gps_sec / 1000.0;
  imt_almanac->header.pc_sec = almanac->header.pc_sec2;
  imt_almanac->header.pc_usec = almanac->header.pc_usec2;

  /* copy the specific value from */
  almanac_sv  = (OEM4_ALMANACB_SV *)&almanac->sv[which_sv];

  imt_almanac->prn = almanac_sv->prn;
  imt_almanac->ecc = almanac_sv->ecc;

  imt_almanac->toa = almanac_sv->seconds;
  /* OEM4 compensate for week wraparound (10 digits, 2^10 = 1024), 
   * OEM3 does not, need to compensate for that since raw ephemeris also
   * wraps around - Ludde */
  imt_almanac->wn  = almanac_sv->week % 1024;
  imt_almanac->OMEGADOT = almanac_sv->omega_dot;
  imt_almanac->OMEGA0   = almanac_sv->omega_o;
  imt_almanac->w   = almanac_sv->w;
  imt_almanac->M0  = almanac_sv->M0;
  imt_almanac->af0 = almanac_sv->af0;
  imt_almanac->af1 = almanac_sv->af1;
  imt_almanac->n   = almanac_sv->n;
  imt_almanac->a   = almanac_sv->a;
  /* OEM4 reports angle of inclination relative to 54 degress 
   * (0.3 semicircles or 0.3 pi. SV's are also braodcasting that value,
   * but OEM3 relcalculates it realtive to 0 degree. Need to compensate
   * for that by adding 0.3 pi - Ludde */
  imt_almanac->i0  = almanac_sv->i0  + 0.3*PI;

  /* dont know if these 3 is correct - Ludde */
  imt_almanac->health4  = almanac_sv->health_prn;
  imt_almanac->health6  = almanac_sv->health_prn;
  imt_almanac->health8  = almanac_sv->health_almanac;

  /* add correct IMT byte length to the message header */
  IMT_MSG_update_length((IMT_MSG *)imt_almanac);
  return 1;
}

void print_OEM4_ALMANAC_and_IMT_ALMANAC(OEM4_ALMANACB_MSG *almanac, 
					IMT_ALMANAC_MSG *imt_almanac, 
					unsigned long which_sv)
{
  printf("OEM4:prn[%i]  week[%i]  seconds[%0.9g]\n",
	 (int)almanac->sv[which_sv].prn,
	 (int)almanac->sv[which_sv].week,
	 almanac->sv[which_sv].seconds);
  printf("IMT :prn[%i]  week[%i]  seconds[%0.9g]\n",
	 (int)imt_almanac->prn,
	 (int)imt_almanac->wn,
	 imt_almanac->toa);
}

int OEM4_repack_EPHEMB_to_IMT_EPHM(OEM4_EPHEMB_MSG *ephem, unsigned char *repb, int rx_nr)
{
  IMT_EPHEMERIS_MSG *imt_ephem;
  double gps_sec;
  int i;

  imt_ephem = (IMT_EPHEMERIS_MSG *)repb;

  /* zero out a IMT_MSG size */
  memset(imt_ephem,0,sizeof(IMT_MSG));

  /* fill in header fields */
  imt_ephem->header.type = IMT_EPHEMERIS;
  imt_ephem->header.rx_nr = (unsigned char)rx_nr;
  imt_ephem->header.rx_type = OEM4_RX;
  imt_ephem->header.status = 0;
  /* The almanac message uses SV_TIME */
  if((ephem->header.gps_stat == GPSTIME_SATTIME))
    imt_ephem->header.time_status = SV_TIME;
  else
    imt_ephem->header.time_status = NO_TIME;
  imt_ephem->header.gps_week = (unsigned long)(ephem->header.gps_week % 1024);
  gps_sec = (double)(ephem->header.millisecs);
  imt_ephem->header.gps_sec = gps_sec / 1000.0;
  imt_ephem->header.pc_sec = ephem->header.pc_sec2;
  imt_ephem->header.pc_usec = ephem->header.pc_usec2;

  
  /* copy prn value */
  imt_ephem->prn = ephem->prn;

  /* copy subframe 1 */
  for(i=0;i<30;i++){
    imt_ephem->subframe[0][i] = ephem->subframe1[i];
  }
  /* copy subframe 2 */
  for(i=0;i<30;i++){
    imt_ephem->subframe[1][i] = ephem->subframe2[i];
  }
  /* copy subframe 3 */
  for(i=0;i<30;i++){
    imt_ephem->subframe[2][i] = ephem->subframe3[i];
  }

  /* add correct IMT byte length to the message header */
  IMT_MSG_update_length((IMT_MSG *)imt_ephem);

  return 1;
}

void print_OEM4_EPHEM_and_IMT_EPEHM(OEM4_EPHEMB_MSG *ephem, 
				    IMT_EPHEMERIS_MSG *imt_ephem)
{
  int i;
  printf("OEM4:prn[%i] IMT: prn[%i]\n",(int)ephem->prn,(int)imt_ephem->prn);

  /* copy subframe 3 */
  for(i=0;i<10;i++){
    printf("1 OEM4:[%x] IMT:[%x]\n", ephem->subframe1[i],imt_ephem->subframe[0][i]);
    printf("2 OEM4:[%x] IMT:[%x]\n", ephem->subframe2[i],imt_ephem->subframe[1][i]);
    printf("3 OEM4:[%x] IMT:[%x]\n", ephem->subframe3[i],imt_ephem->subframe[2][i]);
  }
  
}

int OEM4_repack_SQM_to_IMT_RANGE(OEM4_SQMDATAB_MSG *sqm, unsigned char *msg,int rx_nr)
{
  IMT_SQM_MSG *imt_sqm;
  int obs;
  double gps_sec;
  //always the same for a certain firmwareversion of the OEM4. This is for 1.200s48
  float sqm_corr_space[8] = {-0.025575001,0.025575001,
			     -0.051150002,0.051150002,
			     -0.076725006,0.076725006,
			     -0.102300003,0.10230003}; 
  imt_sqm = (IMT_SQM_MSG *)msg;

  /* zero out a IMT_MSG size */
  memset(imt_sqm,0,sizeof(IMT_MSG));

  /* fill in header fields */
  imt_sqm->header.type = IMT_SQM;
  imt_sqm->header.rx_nr = (unsigned char)rx_nr;
  imt_sqm->header.rx_type = OEM4_RX;
  imt_sqm->header.status = 0;
  /* The SQM message uses GPS time but should not be synchronized, ie use NO_TIME */
  imt_sqm->header.time_status = NO_TIME;

  imt_sqm->header.gps_week = (unsigned long)(sqm->header.gps_week % 1024);
  gps_sec = (double)(sqm->header.millisecs);
  imt_sqm->header.gps_sec = gps_sec / 1000.0;
  imt_sqm->header.pc_sec = sqm->header.pc_sec2;
  imt_sqm->header.pc_usec = sqm->header.pc_usec2;


  imt_sqm->prn =(unsigned long)sqm->prn;
  imt_sqm->nr_corr = 8; // OEM4 specific
  imt_sqm->ch = 0; // should be updated with the correct, shoudl be in oem4 header

  obs =(int)sqm->obs;
  if(obs == 2)
    {// this is a one of ch 0 to 10, i.e it has valid SQM data 
      /* copy the sync-bit, if the correlators are in synch with the master channel */
      if((sqm->ch[0].sync == 1) && (sqm->ch[1].sync == 1))
	imt_sqm->insync = 1;
      else
	imt_sqm->insync = 0;
      
      // -0.025575001
      imt_sqm->corr[0].corr_space = sqm_corr_space[0];
      imt_sqm->corr[0].I = sqm->ch[0].a1sum;
      imt_sqm->corr[0].Q = sqm->ch[0].a2sum;
      // 0.025575001
      imt_sqm->corr[1].corr_space = sqm_corr_space[1];
      imt_sqm->corr[1].I = sqm->ch[0].a4sum;
      imt_sqm->corr[1].Q = 0;
      //-0.051150002
      imt_sqm->corr[2].corr_space = sqm_corr_space[2];
      imt_sqm->corr[2].I = sqm->ch[1].a3sum;
      imt_sqm->corr[2].Q = 0;
      // 0.051150002,
      imt_sqm->corr[3].corr_space = sqm_corr_space[3];
      imt_sqm->corr[3].I = sqm->ch[1].a4sum;
      imt_sqm->corr[3].Q = 0;
      //-0.076725006
      imt_sqm->corr[4].corr_space = sqm_corr_space[4];
      imt_sqm->corr[4].I = sqm->ch[0].a3sum;
      imt_sqm->corr[4].Q = 0;
      // 0.076725006
      imt_sqm->corr[5].corr_space = sqm_corr_space[5];
      imt_sqm->corr[5].I = sqm->ch[0].a5sum;
      imt_sqm->corr[5].Q = 0;

      //-0.102300003
      imt_sqm->corr[6].corr_space = sqm_corr_space[6];
      imt_sqm->corr[6].I = sqm->ch[1].a1sum;
      imt_sqm->corr[6].Q = sqm->ch[1].a2sum;
      // 0.10230003
      imt_sqm->corr[7].corr_space = sqm_corr_space[7];
      imt_sqm->corr[7].I = sqm->ch[1].a5sum;
      imt_sqm->corr[7].Q = 0;
    } else {  /* This is ch 11 or 12, i.e has not valid SQM data */ 
      imt_sqm->nr_corr = 0; // means do not use      
    }
  
  /* add correct IMT byte length to the message header */
  IMT_MSG_update_length((IMT_MSG *)imt_sqm);
  return 1;
}



#ifdef OEM4_CLKB_SATXYZ
/* Repacking fkn to imt messages for CLOCK and SAXYZ are not completed (repacks to OEM3 format) 
 * since no IMT equivalent message type for CLOCK or SATXYZ is defined - Ludde 8-oct-2001 */

// define in the IMT somewhere
#define LIGHT 299792458.0
/* outdated - Ludde 8-oct-2001 */
int OEM4_repack_CLOCK_to_IMT_CLOCK(OEM4_CLOCKB_MSG *clock, unsigned char *clkb)
{

  IMT_CLOCK_MSG  *imt_clock;
  double gps_sec;

  imt_clock = (IMT_CLOCK_MSG *)clkb;

  /* zero out a IMT_MSG size */
  memset(imt_clock,0,sizeof(IMT_MSG));

  /* copy the values the IMT is using  */
  /* OEM4 reports clock offset in meters, IMT uses seconds (OEM3 format) */
  imt_clock->clock_offset = clock->range_bias / LIGHT;

  /* model either valid or not valid for now  */
  if(clock->type == 0)
    imt_clock->clock_model_status = 0;
  else
    imt_clock->clock_model_status = -1;


  /* dont think IMT uses these (clock_drift copied but not used) */
  //double        sa_markov_state;
  //double        std_clock_offset;
  //double        std_clock_drift;

  imt_clock->week = (unsigned long)(clock->header.gps_week % 1024);
  /* OEM4 report time in fixed point ms, IMT uses double and seconds */
  gps_sec = (double)(clock->header.millisecs);
  imt_clock->seconds = gps_sec / 1000.0;
  /* OEM4 reports is sec/sec imt OEM3 uses m/sec */
  imt_clock->clock_drift = clock->drift / LIGHT;

  /* add gps time to the message. Always use GPS_TIME although that might
   * not be true */ 
  IMT_add_gpstime((IMT_MSG *)clkb, imt_clock->seconds, IMT_GPSTIME);

  /* update the IMT header (CRC, msgtype etc) CLKB =  msgtype 2, size 56+12 */
  IMT_update_header((IMT_MSG *)clkb, 2,68-12); 

  return 1;
}

/* outdated - Ludde 8-oct-2001 */
void print_OEM4_CLOCK_and_IMT_CLOCK(OEM4_CLOCKB_MSG *clock,IMT_CLOCK_MSG *imt_clock)
{
  printf("OEM4: offset sec[%0.9g] status[%i] [%0.9g m] \n",
	 clock->range_bias/LIGHT,
	 clock->type,
	 clock->range_bias);

  printf("IMT : offset sec[%0.9g] status[%i]\n",
	 imt_clock->clock_offset,
	 imt_clock->clock_model_status);
} 

/* outdated - Ludde 8-oct-2001 */
int OEM4_repack_SATXYZ_to_IMT_SATXYZ(OEM4_SATXYZB_MSG *satxyz, unsigned char *svdb)
{
  IMT_SATXYZ_MSG  *imt_satxyz;
  IMT_SATXYZ_SV   *imt_satxyz_sv;
  OEM4_SATXYZB_SV  *oem4_satxyz_sv;

  double gps_ms;
  int obs;
  int i;

  imt_satxyz = (IMT_SATXYZ_MSG *)svdb;

  /* zero out a IMT_MSG size*/
  memset(imt_satxyz,0,sizeof(IMT_MSG));

  /* copy the satxyz header variables */
  obs = imt_satxyz->satxyz_header.obs = satxyz->obs;
  imt_satxyz->satxyz_header.weekno    = satxyz->header.gps_week % 1024;
  gps_ms = (double)(satxyz->header.millisecs);
  imt_satxyz->satxyz_header.week_sec  = gps_ms / 1000.0;;
  /* Rx clockerror is not reported in OEM4 satxyz log, setting ot to 0
   * since it is not used in the current IMT code */
  imt_satxyz->satxyz_header.rcerror   = 0.0;

  /* copy SV specific variables */
  for(i=0; i<obs; i++){
    /* make a local reference for more readable code */
    imt_satxyz_sv  = &(imt_satxyz->sv[i]);
    oem4_satxyz_sv = &(satxyz->sv[i]);
    /* copy the variabels */
    imt_satxyz_sv->svprn   = oem4_satxyz_sv->prn;
    imt_satxyz_sv->x       = oem4_satxyz_sv->x;
    imt_satxyz_sv->y       = oem4_satxyz_sv->y;
    imt_satxyz_sv->z       = oem4_satxyz_sv->z;
    imt_satxyz_sv->clkcorr = oem4_satxyz_sv->clk_corr;
    imt_satxyz_sv->Iono    = oem4_satxyz_sv->ion_corr;
    imt_satxyz_sv->Trop    = oem4_satxyz_sv->trop_corr;
    /* No OEM4_SATXYZ variable correspond to these, but should be OK since IMT 
     * code is not using any of these */
    imt_satxyz_sv->Diff_corr    = 0.0;
    imt_satxyz_sv->St_deviation = 0.0;
  }

  /* add gps time to the message. Always use GPS_TIME although that might
   * not be true */ 
  IMT_add_gpstime((IMT_MSG *)svdb,  imt_satxyz->satxyz_header.week_sec, IMT_GPSTIME);

  /* update the IMT header (CRC, msgtype etc) SVDB =  msgtype 36, 
     size 24+12+obs*68 */
  IMT_update_header((IMT_MSG *)svdb, 36, 36-12 + imt_satxyz->satxyz_header.obs*68); 

  return 1;
}

/* outdated - Ludde 8-oct-2001 */
void print_OEM4_SATXYZ_and_IMT_SATXYZ(OEM4_SATXYZB_MSG *satxyz, 
				      IMT_SATXYZ_MSG *imt_satxyz)
{
  int obs;
  int i;
  obs = satxyz->obs;
  printf("SATXYZ Observables OEM4:[%i] IMT:[%i]\n",
	 obs,imt_satxyz->satxyz_header.obs);
  printf("OEM4: GPStime=%u\n",(unsigned int)satxyz->header.millisecs);
  printf("IMT:  GPStime=%0.9g\n",imt_satxyz->satxyz_header.week_sec);
  printf("OEM4: GPSweek=%u\n",(unsigned int)satxyz->header.gps_week);
  printf("IMT:  GPSweek=%u\n",(unsigned int)imt_satxyz->satxyz_header.weekno);
  for(i=0; i<obs; i++){
    printf("OEM4: Prn[%i]  x[%0.9g] y[%0.9g] z[%0.9g]\n",
	   satxyz->sv[i].prn,
	   satxyz->sv[i].x,
	   satxyz->sv[i].y,
	   satxyz->sv[i].z);
    printf("         clk[%0.9g]  iono[%0.9g]  trop[%0.9g]\n",
	   satxyz->sv[i].clk_corr, 
	   satxyz->sv[i].ion_corr, 
	   satxyz->sv[i].trop_corr);

    /* IMT message */
    printf("IMT : Prn[%i]  x[%0.9g] y[%0.9g] z[%0.9g]\n",
	   imt_satxyz->sv[i].svprn,
	   imt_satxyz->sv[i].x,
	   imt_satxyz->sv[i].y,
	   imt_satxyz->sv[i].z);
    printf("         clk[%0.9g]  iono[%0.9g]  trop[%0.9g]\n",
	   imt_satxyz->sv[i].clkcorr, 
	   imt_satxyz->sv[i].Iono, 
	   imt_satxyz->sv[i].Trop);
  }
}

#endif
