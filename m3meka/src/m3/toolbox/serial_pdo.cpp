/*
 * Serial communication over RS232.
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
#include "serial_pdo.h"

namespace m3
{

M3SerialPDO::M3SerialPDO() {

}

M3SerialPDO::~M3SerialPDO() {
}

int M3SerialPDO::GetStatusPdo(unsigned char *pdo_status_raw, std::size_t len)
{
	return port.getLastDataBuffer(pdo_status_raw, len);
}

void M3SerialPDO::SetCommandPdo(unsigned char *pdo_cmd_raw, std::size_t len)
{
	port.sendAsync(pdo_cmd_raw, len);
}

bool M3SerialPDO::Startup(std::string port_name, size_t nb_cmd, size_t nb_status) {	

	//try {
	return port.open(port_name.c_str());
		 // return 
	//} catch (...) {//
		//std::cerr << "Caught an exception while opening port " << port_name << " : " << e.what() << std::endl;
		//return false;
	//}
	//return true;
}

void M3SerialPDO::Shutdown() {
    port.close();
}

} //namespace


