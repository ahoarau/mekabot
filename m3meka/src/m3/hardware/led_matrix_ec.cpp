/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <m3/hardware/led_matrix_ec.h>

namespace m3{
	
using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void M3LedMatrixEc::SetStatusFromPdo(unsigned char * data)
{
	
	if (IsPdoVersion(LED_MATRIX_PDO_V1))
	{
		M3LedMatrixPdoV1Status * ec= (M3LedMatrixPdoV1Status *) data;
		status.set_timestamp(ec->timestamp);
		status.set_flags(ec->flags);
		status.set_debug(ec->debug);
	}
	
}

#define M3LEDMATRIX_EC_LED_MAX 254.0
void M3LedMatrixEc::SetPdoFromCommand(unsigned char * data)
{
	//M3_INFO("En: %d\n",command.enable());
	if (IsPdoVersion(LED_MATRIX_PDO_V1))
	{
		M3LedMatrixPdoV1Cmd * ec = (M3LedMatrixPdoV1Cmd *) data;
		
		ec->enable=(unsigned char)command.enable();
		tmp_cnt++;
		for (int i = 0; i <LED_MTX_BUF_SIZE; i++)
		{
		    int idx=(buf_idx+i)%(nr*nc);
		    int rr = (idx)/GetNumCols();
		    int cc = (idx)%GetNumCols();
		    int r,g,b;
		    r=(int)command.row(rr).column(cc).r();
		    g=(int)command.row(rr).column(cc).g();
		    b=(int)command.row(rr).column(cc).b();
		    
		    r=(int)rgb_slew[rr][cc][0].Step((mReal)r,param.slew_rate());
		    g=(int)rgb_slew[rr][cc][1].Step((mReal)g,param.slew_rate());
		    b=(int)rgb_slew[rr][cc][2].Step((mReal)b,param.slew_rate());
		    
		    mReal sum = r+g+b;
		    if (sum > M3LEDMATRIX_EC_LED_MAX) //overdrive protection
		    {
			r = (mReal)r * (M3LEDMATRIX_EC_LED_MAX/sum);
			g = (mReal)g * (M3LEDMATRIX_EC_LED_MAX/sum);
			b = (mReal)b * (M3LEDMATRIX_EC_LED_MAX/sum);
		    }
		    
		    ec->array[i].r = (unsigned char)r;
		    ec->array[i].g = (unsigned char)g;
		    ec->array[i].b = (unsigned char)b;
		    ec->array[i].idx = (unsigned char)idx;
		    //if (tmp_cnt%100==0)
		    	//M3_INFO("Row: %d Col: %d r: %d g: %d b: %d | en: %d\n",rr,cc,r,g,b,ec->enable);
		}
		buf_idx=(buf_idx+LED_MTX_BUF_SIZE)%(nr*nc);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3LedMatrixEc::ReadConfig(const char * filename)
{	
	if (!M3ComponentEc::ReadConfig(filename))
		return false;	
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	doc["num_rows"] >> nr;
	doc["num_cols"] >> nc;
	mReal val;
	doc["param"]["slew_rate"] >> val;
	param.set_slew_rate(val);
	return true;
}

void M3LedMatrixEc::Startup()
{
	M3ComponentEc::Startup();
	for (int i = 0; i < GetNumRows(); i++)
	{
	  command.add_row();
	  M3LedMatrixEcRGBRow * row = command.mutable_row(i);
	  for (int j = 0; j < GetNumCols(); j++)
	    row->add_column();
	}
	command.set_enable(0);
}

}
   
