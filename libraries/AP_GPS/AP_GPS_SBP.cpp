// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// SBP parser, by Niels Joubert @njoubert
//

/// @file	AP_GPS_SBP.cpp
/// @brief	SBP protocol parser
///
/// Built by Niels Joubert and Fergus Noble

#include <AP_Common.h>
#include <DataFlash.h>
#include <DataFlash.h>
#include <AP_Progmem.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "AP_GPS_SBP.h"

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define SBP_HW_LOGGING 1
#else
#define SBP_HW_LOGGING 0
#endif

#define SBP_MILLIS_BETWEEN_HEALTHCHECKS 1500
#define MILLIS_UNTIL_RTK_DEAD 3000

extern const AP_HAL::HAL& hal;

#define SBP_DEBUGGING 1
#ifdef SBP_DEBUGGING
 //# define Debug(fmt, args ...)  do {printf("GPS_SPB:%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

// Static State    /////////////////////////////////////////////////////////////

bool AP_GPS_SBP::shouldInitState = true;
uint8_t AP_GPS_SBP::_detect_data = 0;
sbp_state_t AP_GPS_SBP::sbp_detect_state;

// Public Methods //////////////////////////////////////////////////////////////


AP_GPS_SBP::AP_GPS_SBP(void) :
  GPS(),
  _last_healthcheck_millis(0),
  _pos_msg_counter(0),
  _crc_error_counter(0),
  _baseline_msg_counter(0),
  _rtk_active(false),
  _rtk_last_baseline_millis(0),
  logging_started(0)
{
}

// Initialize this GPS Driver
// INVATIANTS:
//   - init() might be called multiple times from GPS::update() if no message is received over 1.2s
void AP_GPS_SBP::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting, DataFlash_Class *dataflash)
{
	_port = s;

  Debug("Initializing SBP Driver");
  _port->begin(115200, 256, 16);
  _port->flush();

  _nav_setting = nav_setting;
  _DataFlash = dataflash;

  //First we reset the current SBP State:
  sbp_state_init(&sbp_state);
  sbp_state_set_io_context(&sbp_state, this);

  //Setup all the handlers
  sbp_register_callback(&sbp_state, SBP_GPS_TIME,      &_sbp_callback_gps_time,      this, &this->sbp_callback_node_gps_time);
  sbp_register_callback(&sbp_state, SBP_POS_LLH,       &_sbp_callback_pos_llh,       this, &this->sbp_callback_node_pos_llh);
  sbp_register_callback(&sbp_state, SBP_VEL_NED,       &_sbp_callback_vel_ned,       this, &this->sbp_callback_node_vel_ned);
  sbp_register_callback(&sbp_state, SBP_DOPS,          &_sbp_callback_dops,          this, &this->sbp_callback_node_dops);
  sbp_register_callback(&sbp_state, SBP_BASELINE_ECEF, &_sbp_callback_baseline_ecef, this, &this->sbp_callback_node_baseline_ecef);
  sbp_register_callback(&sbp_state, SBP_BASELINE_NED,  &_sbp_callback_baseline_ned,  this, &this->sbp_callback_node_baseline_ned);

}

bool AP_GPS_SBP::read(void) 
{

  #define GPS_DEBUGGING_FAKE 0
  #if GPS_DEBUGGING_FAKE
    fake_read_pos_llh();
    fake_read_baseline_ecef();
    return true;
  #endif
  //sbp_process does not read all the available data immediately,
  //it might read only one part of the available packet per call 
  //thus, loop while data is available on the port to read entire packets
  bool found_packet = false;
  while (_port->available() > 0) {
    int retval = sbp_process(&sbp_state, &_wrap_sbp_read);
    if (SBP_OK_CALLBACK_EXECUTED == retval || SBP_OK_CALLBACK_UNDEFINED == retval) {
      found_packet = true;
    }
    if (SBP_CRC_ERROR == retval) {
      fix = GPS::FIX_NONE;
      Debug("CRC Errors on GPS!\n");
      _crc_error_counter += 1;
    }
  }  

  uint32_t now = hal.scheduler->millis();
  uint32_t elapsed = now - _last_healthcheck_millis;

  //We do a SBP Healthcheck 
  if (elapsed > SBP_MILLIS_BETWEEN_HEALTHCHECKS) {
    _last_healthcheck_millis = now;

    if (_pos_msg_counter == 0) {
      fix = GPS::FIX_NONE;
    }

    //Calculate some Stats
    #define SBP_CALC_STATS
    #ifdef SBP_CALC_STATS
    float pos_hz = _pos_msg_counter / (float) elapsed * 1000;
    float crc_hz = _crc_error_counter / (float) elapsed * 1000;
    float baseline_hz = _baseline_msg_counter / (float) elapsed * 1000;
    Debug("GPS perfcount: CRC errs: %.2fHz, LLH msgs: %.2fHz, BASELINE msgs: %.2f", crc_hz, pos_hz, baseline_hz);
    #if SBP_HW_LOGGING
    logging_log_health(pos_hz, crc_hz, baseline_hz, _rtk_active, _rtk_has_home);
    #endif
    #endif

    //Reset counters
    _pos_msg_counter = 0;
    _crc_error_counter = 0;
    _baseline_msg_counter = 0;

  }

  //We do a RTK Healthcheck
  if (_rtk_active && ((now - _rtk_last_baseline_millis) > MILLIS_UNTIL_RTK_DEAD)) {
    _rtk_active = false;
    fix = GPS::FIX_NONE;
    Debug("RTK Timeout");
  }

  return found_packet;
}

/*
  detect a SBP GPS.
  Adds one byte, and returns true if the stream
  matches the Swift Binary Protocol
 */
bool AP_GPS_SBP::_detect(uint8_t data) 
{

  //This gets repeatedly called (for all GPS drivers), until one returns true.
  //with one byte of data from the protocol coming in.

  if (shouldInitState) {
    sbp_state_init(&sbp_detect_state);
    sbp_state_set_io_context(&sbp_detect_state, &_detect_data);
    shouldInitState = false;
  }

  //Save the passed-in byte, which will be retrieved by _sbp_detect_read
  _detect_data = data;
  //If the sbp_process ever returns a CALLBACK_EXECUTED or CALLBACK_UNDEFINED then we're good!
  if (sbp_process(&sbp_detect_state, &_sbp_detect_read) > SBP_OK)
    return true;

  return false;
}

//Returns a status code:
// 0 == okay
// 1 == no SPP solution
// 2 == no RTK baseline
int AP_GPS_SBP::capture_as_home() 
{

  if (fix < FIX_3D) {
    Debug("Attempt to capture home without GPS Fix available. Can't do RTK, refusing to capture home position");
    return 1;
  }

  if (!_rtk_active) {
    Debug("Attempting to capture home without RTK baseline available. Can't do RTK, refusing to capture home position");
    return 2;
  }

  //INVARIANT: This should NOT cause the current position to jump

  double homeLLH[3];
  double homeBaselineECEF[3];  
  double homeECEF[3];

  //Grab the current position.
  //NOTE: If we have a current RTK-based position, it'll reuse this, else it'll be SPP
  //convert to degrees and meters.
  homeLLH[0] = ((double)latitude) / 10000000.0;
  homeLLH[1] = ((double)longitude) / 10000000.0;
  homeLLH[2] = ((double)altitude_cm) / 100.0;

  //convert LLH to ECEF
  llhdeg2rad(homeLLH, homeLLH);
  wgsllh2ecef(homeLLH, homeECEF);

  //Grab the current baseline
  //convert to meters.
  homeBaselineECEF[0] = ((double)_rtk_last_baselineECEF[0])/1000.0;
  homeBaselineECEF[1] = ((double)_rtk_last_baselineECEF[1])/1000.0;
  homeBaselineECEF[2] = ((double)_rtk_last_baselineECEF[2])/1000.0;

  //Calculate home reference point.
  vector_subtract(3, homeECEF, homeBaselineECEF, _rtk_referencePointECEF);

  //Save the current baseline, save the current home position.
  Debug("Captured Home State. _rtk_referencePointECEF = (%f, %f, %f)",
     _rtk_referencePointECEF[0],
     _rtk_referencePointECEF[1],
     _rtk_referencePointECEF[2]);

  _rtk_has_home = true;
  return 0;
}

//Called from the wrapper function to read data from this port.
uint32_t AP_GPS_SBP::sbp_read(uint8_t* buff, uint32_t n) 
{
  int32_t numc = min(_port->available(), n);
  for (int32_t i = 0; i < n; i++) {
    buff[i] = _port->read();
  }
  return numc;
}

void AP_GPS_SBP::read_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[]) 
{

  //todo(njoubert): Potentially, if we haven't gotten a POS/VEL packet in a while, we wanna set fix to None.

  sbp_gps_time_t* t = (sbp_gps_time_t*)msg;

  time_week         = t->wn;
  time_week_ms      = t->tow;

}
void AP_GPS_SBP::read_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[]) 
{
 
  sbp_pos_llh_t* p   = (sbp_pos_llh_t*)msg;

  time_week_ms   = p->tow;
  
  _sp_latitude       = (int32_t) (p->lat*1e7); /* Convert decimal degrees to degrees*10000000 */
  _sp_longitude      = (int32_t) (p->lon*1e7); /* Convert decimal degrees to degrees*10000000 */
  _sp_altitude_cm    = (int32_t) (p->height*1e2);   /* Convert meters to cm. */


  num_sats       = p->n_sats;

  _pos_msg_counter += 1;
//  Debug("Received pos message, lat=%f (%d) lon=%f (%d) alt=%f (%d) num_sats=%d", p->lat, latitude, p->lon, longitude, p->height, altitude_cm, num_sats);
  update_public_lat_lon_alt();
}
void AP_GPS_SBP::read_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[]) 
{

  _have_raw_velocity = true;

  sbp_vel_ned_t* v   = (sbp_vel_ned_t*)msg;

  // velocities in cm/s if available from the GPS
  time_week_ms       = v->tow;
  _vel_north         = (int32_t) (v->n/10); /* Convert mm/s to cm/s */
  _vel_east          = (int32_t) (v->e/10); /* Convert mm/s to cm/s */
  _vel_down          = (int32_t) (v->d/10); /* Convert mm/s to cm/s */
  num_sats           = v->n_sats;

  int32_t ground_vector_sq = _vel_north*_vel_north + _vel_east*_vel_east;
  ground_speed_cm      = safe_sqrt(ground_vector_sq);                                      ///< ground speed in cm/sec
  //speed_3d_cm          = safe_sqrt(_vel_down*_vel_down + ground_vector_sq);  ///< 3D speed in cm/sec (not always available)

  ground_course_cd = (int32_t) 100*ToDeg(atan2f(_vel_east, _vel_north));
  if (ground_course_cd < 0) {
      ground_course_cd += 36000;
  }

}

void AP_GPS_SBP::read_dops(uint16_t sender_id, uint8_t len, uint8_t msg[]) 
{

  sbp_dops_t* d   = (sbp_dops_t*)msg;

  time_week_ms = d->tow;
  hdop = d->hdop;

}

void AP_GPS_SBP::read_baseline_ecef(uint16_t sender_id, uint8_t len, uint8_t msg[]) 
{
  
  sbp_baseline_ecef_t* b = (sbp_baseline_ecef_t*)msg;

  time_week_ms                = b->tow;
  _rtk_last_baselineECEF[0]   = b->x;
  _rtk_last_baselineECEF[1]   = b->y;
  _rtk_last_baselineECEF[2]   = b->z;
  _rtk_last_baseline_accuracy = b->accuracy;
  num_sats                    = b->n_sats;

  //Activate RTK
  _rtk_last_baseline_millis = hal.scheduler->millis();
  if (!_rtk_active) {
    Debug("Received ECEF Baseline: Activating RTK");
  }
  _rtk_active = true;
  _baseline_msg_counter += 1;

  update_public_lat_lon_alt();

}

void AP_GPS_SBP::read_baseline_ned(uint16_t sender_id, uint8_t len, uint8_t msg[]) 
{
  //We consciously do not support NED baselines, Piksi should output ECEF.
  //NED needs to be rotated into ECEF, which takes extra processing power,
  //and Piksi natively works in ECEF, thus deny using NED.
  //Notify the user if he's sending NED and not ECEF.  
  if (!_rtk_active) {
    Debug("ERROR: Received NED Baseline, but no ECEF baseline");
  }
}

void AP_GPS_SBP::fake_read_pos_llh() {

  time_week_ms   = 100;
  
  _sp_latitude       = 377833000; /* Convert decimal degrees to degrees*10000000 */
  _sp_longitude      = 1224167000; /* Convert decimal degrees to degrees*10000000 */
  _sp_altitude_cm    = 100;   /* Convert meters to cm. */


  num_sats       = 9;

  _pos_msg_counter += 1;
//  Debug("Received pos message, lat=%f (%d) lon=%f (%d) alt=%f (%d) num_sats=%d", p->lat, latitude, p->lon, longitude, p->height, altitude_cm, num_sats);
  update_public_lat_lon_alt();


}
void AP_GPS_SBP::fake_read_baseline_ecef() {

  time_week_ms                = 100;
  _rtk_last_baselineECEF[0]   = 100;
  _rtk_last_baselineECEF[1]   = 100;
  _rtk_last_baselineECEF[2]   = 100;
  _rtk_last_baseline_accuracy = 1;
  num_sats                    = 9;

  //Activate RTK
  _rtk_last_baseline_millis = hal.scheduler->millis();
  if (!_rtk_active) {
    Debug("Received ECEF Baseline: Activating RTK");
  }
  _rtk_active = true;
  _baseline_msg_counter += 1;

  _rtk_active = true;

  update_public_lat_lon_alt();


}


void AP_GPS_SBP::update_public_lat_lon_alt() {

  if (_rtk_active && _rtk_has_home) {

    double currentBaselineECEF[3];
    currentBaselineECEF[0] = ((double)_rtk_last_baselineECEF[0])/1000.0;
    currentBaselineECEF[1] = ((double)_rtk_last_baselineECEF[1])/1000.0;
    currentBaselineECEF[2] = ((double)_rtk_last_baselineECEF[2])/1000.0;

    double currentPositionECEF[3];
    vector_add(3, _rtk_referencePointECEF, currentBaselineECEF, currentPositionECEF);

    double currentPositionLLH[3];
    wgsecef2llh(currentPositionECEF, currentPositionLLH);

    llhrad2deg(currentPositionLLH, currentPositionLLH);

    latitude    = (int32_t) (currentPositionLLH[0] * 1e7);
    longitude   = (int32_t) (currentPositionLLH[1] * 1e7);
    altitude_cm = (int32_t) (currentPositionLLH[2] * 1e2);
    
    if (fix != GPS::FIX_RTK) {
      Debug("Switching to RTK");
    }
    fix = GPS::FIX_RTK;


    //Debug("RTK lat, lon, alt: %d %d %d",latitude, longitude,altitude_cm);

  } else {

    //Just update based on Single Point Positioning
    latitude  = _sp_latitude;
    longitude = _sp_longitude;
    altitude_cm  =_sp_altitude_cm;
    if (fix != GPS::FIX_3D) {
      Debug("Switching to SPP");
    }
    fix = GPS::FIX_3D;

    //Debug("SPP lat, lon, alt: %d %d %d",latitude, longitude,altitude_cm);

  }
}


#if SBP_HW_LOGGING

#define LOG_MSG_SBP1 202
#define LOG_MSG_SBP2 203

struct PACKED log_Sbp1 {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float    pos_hz;
    float    crc_hz;
    float    baseline_hz;
    uint8_t  rtk_active;
    uint8_t  rtk_has_home;
};


static const struct LogStructure sbp_log_structures[] PROGMEM = {
    { LOG_MSG_SBP1, sizeof(log_Sbp1),
      "SBP1", "Ifffbb",  "TimeMS,PosHz,CrcHz,BaselienHz,rtkActive,rtkHasHome" },
};

void AP_GPS_SBP::logging_write_headers(void)
{
    if (!logging_started) {
        logging_started = true;
        _DataFlash->AddLogFormats(sbp_log_structures, 1);
    }
}

void AP_GPS_SBP::logging_log_health(float pos_hz, float crc_hz, float baseline_hz, bool rtk_active, bool rtk_has_home)
{

  if (_DataFlash == NULL || !_DataFlash->logging_started()) {
      return;
  }

  logging_write_headers();

  struct log_Sbp1 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_MSG_SBP1),
    timestamp    : hal.scheduler->millis(),
    pos_hz       : pos_hz,
    crc_hz       : crc_hz,
    baseline_hz  : baseline_hz,
    rtk_active   : rtk_active,
    rtk_has_home : rtk_has_home,
  };
  _DataFlash->WriteBlock(&pkt, sizeof(pkt));    

}


#endif // SBP_HW_LOGGING


/* 
  
  Define static callback functions that pipes through C-style callbacks into C++ objects

*/

// This function is passed as a function pointer to the sbp_process function.
// It is called by sbp_process to read bytes into a buffer, which sbp_process
// uses to decode messages.
uint32_t _wrap_sbp_read(uint8_t* buff, uint32_t n, void* context) 
{
  return ((AP_GPS_SBP*)context)->sbp_read(buff, n);
}

void _sbp_callback_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) 
{
  ((AP_GPS_SBP*)context)->read_gps_time(sender_id, len, msg);
}

void _sbp_callback_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) 
{
  ((AP_GPS_SBP*)context)->read_pos_llh(sender_id, len, msg);
}

void _sbp_callback_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) 
{
  ((AP_GPS_SBP*)context)->read_vel_ned(sender_id, len, msg);
}

void _sbp_callback_dops(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) 
{
  ((AP_GPS_SBP*)context)->read_dops(sender_id, len, msg);
}

void _sbp_callback_baseline_ecef(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) 
{
  ((AP_GPS_SBP*)context)->read_baseline_ecef(sender_id, len, msg);
}

void _sbp_callback_baseline_ned(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) 
{
  ((AP_GPS_SBP*)context)->read_baseline_ned(sender_id, len, msg);
}

static uint32_t _sbp_detect_read(uint8_t* buff, uint32_t n, void* context) {
  *buff = *((uint8_t*)context);
  return 1;
}


