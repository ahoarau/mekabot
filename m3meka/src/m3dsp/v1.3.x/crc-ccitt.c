//
// crc-ccitt
//
// Copyright 2007 Dean Ferreyra, All rights reserved
//
// This file is part of dspic-helper.
//
// dspic-helper is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// dspic-helper is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with dspic-helper.  If not, see <http://www.gnu.org/licenses/>.
//
// $Id: crc-ccitt.c,v 1.1 2008-11-12 22:39:59 dean Exp $

//
// Translated from http://www.joegeluso.com/software/articles/ccitt.htm
//

#include <boolean.h>
#include "crc-ccitt.h"

uint16_t crc_ccitt(uint16_t crc, char byte)
{
    uint8_t v = 0x80;
    for (int i = 0; i < 8; i++) {
        bool xor_p = crc & 0x8000;
        crc <<= 1;
        if (byte & v) {
            crc += 1;
        }
        if (xor_p) {
            crc ^= CRC_CCITT_POLYNOMIAL;
        }
        v >>= 1;
    }
    return crc;
}

uint16_t crc_ccitt_normalize(uint16_t crc)
{
    for (int i = 0; i < 16; i++) {
        bool xor_p = crc & 0x8000;
        crc <<= 1;
        if (xor_p) {
            crc ^= CRC_CCITT_POLYNOMIAL;
        }
    }
    return crc;
}
