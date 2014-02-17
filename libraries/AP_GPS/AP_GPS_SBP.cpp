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

#include <AP_Progmem.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "AP_GPS_SBP.h"

extern const AP_HAL::HAL& hal;

#define SBP_DEBUGGING
#ifdef SBP_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("GPS_SPB:%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif


// Public Methods //////////////////////////////////////////////////////////////

// Initialize this GPS Driver
// INVATIANTS:
//   - init() might be called multiple times from GPS::update() if no message is received over 1.2s
void AP_GPS_SBP::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
  Debug("Calling INIT() on SBP GPS driver.");

  //First we reset the current SBP State:
  sbp_state_init(&sbp_state);
  sbp_state_set_io_context(&sbp_state, this);

  //Setup all the handlers
  sbp_register_callback(&sbp_state, SBP_GPS_TIME, &_sbp_callback_gps_time, this, &this->sbp_callback_node_gps_time);
  sbp_register_callback(&sbp_state, SBP_POS_LLH,  &_sbp_callback_pos_llh,  this, &this->sbp_callback_node_pos_llh);
  sbp_register_callback(&sbp_state, SBP_VEL_NED,  &_sbp_callback_vel_ned,  this, &this->sbp_callback_node_vel_ned);

}


s8 sbp_register_callback(sbp_state_t* s, u16 msg_type, sbp_msg_callback_t cb, void* context,
                         sbp_msg_callbacks_node_t *node);

bool AP_GPS_SBP::read(void) {

  //sbp_process does not read all the available data immediately,
  //it might read only one part of the available packet per call 
  //thus, loop while data is available on the port to read entire packets
  bool found_packet = false;
  while (_port->available() > 0) {
    if (SBP_CALLBACK_EXECUTED == sbp_process(&sbp_state, &_wrap_sbp_read)) {
      found_packet = true;
    }
  }  

  return found_packet;
}

/*
  detect a SBP GPS.
 */
bool AP_GPS_SBP::_detect(uint8_t data) {

  Debug("For the moment we always return true for SBP Detection");

  //This gets repeatedly called (for all GPS drivers), until one returns true.
  //with one byte of data from the protocol coming in.


  return true;
}

//Called from the wrapper function to read data from this port.
uint32_t AP_GPS_SBP::sbp_read(uint8_t* buff, uint32_t n) {
  int32_t numc = min(_port->available(), n);
  for (int32_t i = 0; i < n; i++) {
    buff[i] = _port->read();
  }
  return numc;
}

void AP_GPS_SBP::read_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[]) {
  sbp_gps_time_t* t = (sbp_gps_time_t*)msg;
  Debug("read a GPS Time message with tow=%d\n", t->tow);
}
void AP_GPS_SBP::read_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[]) {

}
void AP_GPS_SBP::read_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[]) {

}


//Define the private state /////////////////////////////////////////////////////



// This function is passed as a function pointer to the sbp_process function.
// It is called by sbp_process to read bytes into a buffer, which sbp_process
// uses to decode messages.
uint32_t _wrap_sbp_read(uint8_t* buff, uint32_t n, void* context) {
  
  return ((AP_GPS_SBP*)context)->sbp_read(buff, n);

}

void _sbp_callback_gps_time(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) {
  ((AP_GPS_SBP*)context)->read_gps_time(sender_id, len, msg);
}
void _sbp_callback_pos_llh(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) {
  ((AP_GPS_SBP*)context)->read_pos_llh(sender_id, len, msg);
}
void _sbp_callback_vel_ned(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context) {
  ((AP_GPS_SBP*)context)->read_vel_ned(sender_id, len, msg);
}

