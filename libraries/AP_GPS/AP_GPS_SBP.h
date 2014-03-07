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
#include <libswiftnav/sbp.h>
#include <libswiftnav/sbp_messages.h>
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
    virtual void        init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting = GPS_ENGINE_NONE);

    /// Checks the serial receive buffer for characters,
    /// attempts to parse NMEA data and updates internal state
    /// accordingly.
    ///
    virtual bool        read();

    static bool _detect(uint8_t data);

    uint32_t sbp_read(uint8_t* buff, uint32_t n);

    void read_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[]);
    void read_dops(uint16_t sender_id, uint8_t len, uint8_t msg[]);

private:

  //SBP Parser State and Callbacks
  sbp_state_t sbp_state;
  sbp_msg_callbacks_node_t sbp_callback_node_gps_time;
  sbp_msg_callbacks_node_t sbp_callback_node_pos_llh;
  sbp_msg_callbacks_node_t sbp_callback_node_vel_ned;
  sbp_msg_callbacks_node_t sbp_callback_node_dops;

  //statistics
  uint32_t _last_healthcheck_millis;
  uint32_t _pos_msg_counter;
  uint32_t _crc_error_counter;

  //SBP Parser for Detecting Messages
  static bool shouldInitState;
  static uint8_t _detect_data;
  static sbp_state_t sbp_detect_state;

};


//FILE-PRIVATE WRAPPER FUCTIONS FOR C CALLBACK FUNCTIONS
static uint32_t _wrap_sbp_read(uint8_t* buff, uint32_t n, void* context);
static void _sbp_callback_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
static void _sbp_callback_dops(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);

static uint32_t _sbp_detect_read(uint8_t* buff, uint32_t n, void* context);

#endif // __AP_GPS_SBP_H_
