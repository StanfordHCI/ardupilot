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
// NMEA parser, adapted by Michael Smith from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
//

/// @file	AP_GPS_SBP.h
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.  It is frugal in its use of memory
/// and tries to avoid unnecessary arithmetic.
///
/// The parser handles GPGGA, GPRMC and GPVTG messages, and attempts to be
/// robust in the face of occasional corruption in the input stream.  It
/// makes a basic effort to configure GPS' that are likely to be connected in
/// NMEA mode (SiRF, MediaTek and ublox) to emit the correct message
/// stream, but does not validate that the correct stream is being received.
/// In particular, a unit emitting just GPRMC will show as having a fix
/// even though no altitude data is being received.
///
/// GPVTG data is parsed, but as the message may not contain the the
/// qualifier field (this is common with e.g. older SiRF units) it is
/// not considered a source of fix-valid information.
///


#ifndef __AP_GPS_SBP_H_
#define __AP_GPS_SBP_H_

#include <AP_HAL.h>
#include "GPS.h"
#include <AP_Progmem.h>


#define SBP_HEADER_1  0xBE
#define SBP_HEADER_2  0xEF

enum sbp_state_state_t {
    WAITING_1 = 0,
    WAITING_2,
    GET_TYPE,
    GET_LEN,
    GET_MSG,
    GET_CRC
};

/** State structure for processing SBP messages from a particular USART. */
typedef struct {
 sbp_state_state_t state;
  uint8_t msg_type;
  uint8_t msg_len;
  uint8_t msg_n_read;
  uint8_t msg_buff[256];
  uint8_t crc_n_read;
  uint8_t crc[2];
} sbp_process_messages_state_t;


/// SBP parser
///
class AP_GPS_SBP : public GPS
{
public:
	AP_GPS_SBP(void) : 
	GPS()
    	{}

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

private:

    void sbp_process_usart();

    sbp_process_messages_state_t sbp_state;

};

#endif // __AP_GPS_SBP_H_
