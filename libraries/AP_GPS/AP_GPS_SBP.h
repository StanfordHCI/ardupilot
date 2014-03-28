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


#ifndef __AP_GPS_SBP_H_
#define __AP_GPS_SBP_H_

#include <AP_HAL.h>
#include "GPS.h"
#include <AP_Progmem.h>

extern "C" {
#include "sbp.h"
#include "sbp_messages.h"
#include "coord_system.h"
#include "linear_algebra.h"
}

/// SBP parser
///
class AP_GPS_SBP : public GPS
{
public:
	AP_GPS_SBP(void);

    /// Perform a (re)initialisation of the GPS; sends the
    /// protocol configuration messages.
    ///
    virtual void        init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting, DataFlash_Class *dataflash);

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    ///
    virtual bool        read();

    static bool _detect(uint8_t data);

    int capture_as_home();

    uint32_t sbp_read(uint8_t* buff, uint32_t n);

    void read_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_dops(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_baseline_ecef(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_baseline_ned(uint16_t sender_id, uint8_t len, uint8_t msg[]);


    //For testing purposes.
    void fake_read_pos_llh();
    void fake_read_baseline_ecef();

private:

  //RTK-related variables to support reporting absolute positions
  //by calculating centimeter-accurate offsets from fixed home lat-lon.
  //
  //  INITIALIZATION:     Given homeLatLon, homeBaselineFromRef, we calculate referencePointECEF
  //  ONLINE CALCULATION: Given baselineFromRef, we calculate currentLocationECEF.
  //
  // Since we hold the home point constant, 
  // we output a lat-lon that is only GPS-accurate absolutely, 
  // but RTK-accurate relative to home
  //
  // See https://github.com/swift-nav/libswiftnav/raw/master/docs/sbp.pdf
  //

  //RTK STATE MACHINE:
  // state  IN_SPP_STATE:    when !(_rtk_messages_incoming && _rtk_has_home)
  // state  IN_RTK_STATE:   when _rtk_messages_incoming && _rtk_has_home

  bool      _rtk_active;      //Are we receiving RTK messages?
  bool      _rtk_has_home;    //Do we have a home position to do pseudo-absolute positioning with?
  uint32_t  _rtk_last_baseline_millis;

  int32_t   _rtk_last_baselineECEF[3]; //Last received baseline, in mm.
  uint16_t  _rtk_last_baseline_accuracy;
  double    _rtk_referencePointECEF[3]; //Reference point in absolute ECEF coordinates

  //single-point solutions
  int32_t _sp_latitude;                   ///< latitude in degrees * 10,000,000
  int32_t _sp_longitude;                  ///< longitude in degrees * 10,000,000
  int32_t _sp_altitude_cm;                ///< altitude in cm

  //RTK State Machine Transition Side-effects:
  //update the publically-visible lat, lon, alt variables either from RTK or SPP variables
  void update_public_lat_lon_alt();

  //SBP Parser State and Callbacks
  sbp_state_t sbp_state;
  sbp_msg_callbacks_node_t sbp_callback_node_gps_time;
  sbp_msg_callbacks_node_t sbp_callback_node_pos_llh;
  sbp_msg_callbacks_node_t sbp_callback_node_vel_ned;
  sbp_msg_callbacks_node_t sbp_callback_node_dops;
  sbp_msg_callbacks_node_t sbp_callback_node_baseline_ecef;
  sbp_msg_callbacks_node_t sbp_callback_node_baseline_ned;

  //statistics
  uint32_t _last_healthcheck_millis;
  uint32_t _pos_msg_counter;
  uint32_t _baseline_msg_counter;
  uint32_t _crc_error_counter;

  //SBP Parser for Detecting Messages
  static bool shouldInitState;
  static uint8_t _detect_data;
  static sbp_state_t sbp_detect_state;

  // have we written the logging headers to DataFlash?
  bool            logging_started:1;

  void logging_write_headers();
  void logging_log_health(float pos_hz, float crc_hz, float baseline_hz, bool rtk_active, bool rtk_has_home);

};


//FILE-PRIVATE WRAPPER FUCTIONS FOR C CALLBACK FUNCTIONS
static uint32_t _wrap_sbp_read(uint8_t* buff, uint32_t n, void* context);
static void _sbp_callback_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_dops(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_baseline_ecef(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_baseline_ned(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);

static uint32_t _sbp_detect_read(uint8_t* buff, uint32_t n, void* context);


#endif // __AP_GPS_SBP_H_
