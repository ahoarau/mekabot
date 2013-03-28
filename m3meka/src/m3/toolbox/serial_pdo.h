/*
 * M3 Component to abstract PDO over serial (rather than ethercat)
 * Copyright (C) 2011  Meka Robotics
 * Author: edsinger@mekabot.com (Aaron Edsinger)
 * Author <pierrelucbacon@mekabot.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef M3SERIALPDO_H_
#define M3SERIALPDO_H_

#include "serial_port.h"

namespace m3
{

class M3SerialPDO {
public:
	M3SerialPDO();
	virtual ~M3SerialPDO();

	/**
	 * Get the most recent status PDO received from the device.
	 * This call is meant to be non-blocking.
	 * @param pdo_status_raw The memory location in which to copy the PDO
	 * that was received. pdo_status_raw points to NULL if no status PDO
	 * is available.
	 * Returns number of bytes copied. 
	 */
	int GetStatusPdo(unsigned char * pdo_status_raw, std::size_t len);

	/**
	 * Send a PDO command to the device.
	 * This call is meant to be non-blocking.
	 * @param pdo_cmd_raw A pointer to PDO structure to send to the device.
	 */
	void SetCommandPdo(unsigned char * pdo_cmd_raw, std::size_t len);

	bool Startup(std::string port_name, size_t nb_cmd, size_t nb_status);

	void Shutdown();

private:
	M3SerialPort port;
};

}//namespace

#endif /* M3SERIALPDO_H_ */

