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



/* CRC16 implementation acording to CCITT standards */
static const uint16_t crc16tab[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/** Calculate CCITT 16-bit Cyclical Redundancy Check (CRC16).
 *
 * This implementation uses parameters used by XMODEM i.e. polynomial is:
 * \f[
 *   x^{16} + x^{12} + x^5 + 1
 * \f]
 * Mask 0x11021, not reversed, not XOR'd
 * (there are several slight variants on the CCITT CRC-16).
 *
 * \param buf Array of data to calculate CRC for
 * \param len Length of data array
 * \param crc Initial CRC value
 *
 * \return CRC16 value
 */
uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc)
{
  for (uint32_t i = 0; i < len; i++)
    crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *buf++) & 0x00FF];
  return crc;
}


// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_SBP::init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting)
{
	_port = s;
}

bool AP_GPS_SBP::read(void)
{
    


    // if (_port->available()) {
    //   int16_t inByte = _port->read();
    //   hal.console->write(inByte);
    // }
    // return false;

  sbp_process_usart();

}



/** Process SBP messages for a particular USART.
 * Extract data from the USART DMA buffer and parse SBP messages in the data.
 * When a valid SBP message is found (callback is registered for the message
 * ID and CRC is valid), call the callback function for it.
 *
 * \param s sbp_process_messages_state_t pointer of the USART to process.
 */
void AP_GPS_SBP::sbp_process_usart()
{
  uint8_t len, temp;
  uint16_t crc, crc_rx;

  while ((len = _port->available())) {
    /* If there are no bytes waiting to be processed then return. */
    if (len == 0)
      return;

    switch (sbp_state.state) {
    case WAITING_1:
      temp = _port->read();
      if (temp == SBP_HEADER_1)
        sbp_state.state = WAITING_2;
      break;

    case WAITING_2:
      temp = _port->read();
      if (temp == SBP_HEADER_2)
        sbp_state.state = GET_TYPE;
      else
        sbp_state.state = WAITING_1;
      break;

    case GET_TYPE:
      sbp_state.msg_type = _port->read();
      sbp_state.state = GET_LEN;
      break;

    case GET_LEN:
      sbp_state.msg_len = _port->read();
      sbp_state.msg_n_read = 0; 
      sbp_state.state = GET_MSG;
      break;

    case GET_MSG:
      if (sbp_state.msg_len - sbp_state.msg_n_read > 0) {
        /* Not received whole message yet, try and get some more. */
        sbp_state.msg_buff[sbp_state.msg_n_read] = _port->read();
        sbp_state.msg_n_read += 1;
      } else {
        sbp_state.crc_n_read = 0;
        sbp_state.state = GET_CRC;
      }
      break;

    case GET_CRC:
      if (sbp_state.crc_n_read < 2) {
        sbp_state.crc[sbp_state.crc_n_read] = _port->read();
        sbp_state.crc_n_read += 1;

      } else {
        crc = crc16_ccitt(&(sbp_state.msg_type), 1, 0);
        crc = crc16_ccitt(&(sbp_state.msg_len), 1, crc);
        crc = crc16_ccitt(sbp_state.msg_buff, sbp_state.msg_len, crc);
        crc_rx = (sbp_state.crc[0]) |
                 ((sbp_state.crc[1] & 0xFF) << 8);
        if (crc_rx == crc) {
          /* Message complete, process it. */
          hal.console->printf("Message Type %02x\n", sbp_state.msg_type);
        } else
          hal.console->printf("CRC error 0x%04X 0x%04X\n", crc, crc_rx);
        sbp_state.state = WAITING_1;
      }
      break;

    default:
      sbp_state.state = WAITING_1;
      break;
    }
  }
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
