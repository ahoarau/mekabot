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


#include "toolbox.h"

namespace m3
{
using namespace std;
	mReal EvalCalibrationPoly(vector<mReal> & p, mReal x)
	{
		mReal val=0;
		int idx=0;
		int n=(int)p.size();
		if (n==0)
			return 0.0;
		for (int nn=n-1;nn>=0;nn--)
			val += p[idx++] * (pow(x,nn));
		return val;
	}

	mReal M3PID::Step(mReal sense, mReal sense_dot,mReal des, mReal p, mReal i, 
					   mReal d, mReal i_limit, mReal i_range)
	{		
		e=sense-des;
		p_term = p*e;
		//Limit windup outside of a range
		//However, allow it to regain if windups in one direction
		//Overshoots to opposite of range.
		if (ABS(e)<i_range || SIGN(i_error_sum)!=SIGN(i*e))
			i_error_sum=CLAMP(i_error_sum+i*e, -i_limit,i_limit);
		if (i==0)
			i_error_sum=0;
		i_term = i_error_sum;
		d_term = d * sense_dot;
		return p_term+i_term+d_term;
	}
	
	void M3PID::ResetIntegrator()
	{
	    i_error_sum = 0;
	    
	}
	
}//namespace



