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
// $Id: crc-ccitt.h,v 1.2 2008-11-13 08:34:55 dean Exp $
//
// Translated from http://www.joegeluso.com/software/articles/ccitt.htm
//
	
#include <inttypes.h>
	
#define CRC_CCITT_POLYNOMIAL 0x1021

// Start by seeding the CRC value with CRC_CCITT_INITIAL_VALUE.  As a
// final step, call crc_ccitt_normalize to get the actual CRC value.

#define CRC_CCITT_INITIAL_VALUE 0xFFFF
	
uint16_t crc_ccitt(uint16_t crc, char byte);
uint16_t crc_ccitt_normalize(uint16_t crc);
