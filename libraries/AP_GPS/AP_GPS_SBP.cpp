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

/// @file	AP_GPS_SBP.cpp
/// @brief	SBP protocol parser
///
/// Built by Niels Joubert and Fergus Noble

#include <AP_Common.h>

#include <AP_Progmem.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>

#include "AP_GPS_SBP.h"

extern const AP_HAL::HAL& hal;



// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_SBP::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
}

bool AP_GPS_SBP::read(void)
{
    if (_port->available()) {
      int16_t inByte = _port->read();
      hal.console->write(inByte);
    }
    return false;
}


/*
  detect a SBP GPS.
 */
bool
AP_GPS_SBP::_detect(uint8_t data)
{
    //TODO: Make this work
    return false;
}
