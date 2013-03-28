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

#ifndef M3_TOOLBOX_H
#define M3_TOOLBOX_H


#include <vector>
#include <math.h>
#include <string>
#include <stdint.h>
#include <cstdlib>
#include <climits>
#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/toolbox.h>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames_io.hpp>

namespace m3
{

	using namespace std;
	using namespace m3rt;
class M3Avg
{
public:
	M3Avg(){}
	M3Avg(int n){Resize(n);}
	mReal Step(mReal d){
		if (idx>=buf.size())
		{
			M3_INFO("Invalid buffer size for M3Avg\n");
			return 0;
		}
		val=val-buf[idx];
		buf[idx]=d;
		val=val+d;
		idx=(idx+1)%(int)buf.size();
		return val/(int)buf.size();
	}
	
	void Resize(int n){
		if (buf.size()!=n)
		{
			idx=0;
			val=0;
			buf.resize(n,0);
		}
	}
private:
	vector<mReal> buf;
	int idx;
	mReal val;
};


//Average the value over a time window (uS)
//Assuming that Step is called at a fixed period (uS)
class M3TimeAvg
{
	public:
		M3TimeAvg():idx(0),val(0),cntr(0) {}
		void Resize(int window_us, int downsample)
		{
			ds=MAX(1,downsample);
			int period_us = RT_TIMER_TICKS_NS/1000;
			int step_size_us=period_us*downsample;
			int nsteps_per_window = (window_us/step_size_us)+1;
			buf.resize(nsteps_per_window,0);
		}
		void Reset(mReal v)
		{
			for (int i=0;i<buf.size();i++)
				buf[i]=v;
			val=v*(mReal)buf.size();
		}
		mReal Step(mReal d){
			if (idx>=buf.size())
			{
				M3_INFO("Invalid buffer size for M3TimeAvg\n");
				return 0;
			}
			if (cntr++%ds==0)
			{
				//if (d=.00001)
				//	M3_INFO("D: %f, Val: %f, idx: %d, Buf: %f Sz: %d\n",d,val,idx,buf[idx],buf.size());
				val=val-buf[idx];
				buf[idx]=d;
				val=val+d;
				idx=(idx+1)%(int)buf.size();
				return val/(mReal)buf.size();
			}
			return val/(mReal)buf.size();
		}
	private:
		vector<mReal> buf;
		int idx;
		mReal val;
		int ds;
		int cntr;

};

//RC filter, where:
// y_k = (T/(T+h)) y_k-1 + (h/(T+h)) x_k
class M3ExponentialAvg
{
	public:
		M3ExponentialAvg():y_k(0),y_k_last(0){}
		void Resize(mReal sample_period_s, mReal time_constant_s)
		{
			T=time_constant_s;
			h=sample_period_s;
		}
		void Reset(mReal val)
		{
			y_k=val;
			y_k_last=val;
		}
		mReal Step(mReal x_k){
			y_k = (T/(T+h))*y_k_last + (h/(T+h))*x_k;
			y_k_last=y_k;
			return y_k;
		}
	private:
		mReal y_k;
		mReal y_k_last;
		mReal h;
		mReal T;
};

//Slew to a desired value at a given rate
//Units are in S, of if value is in degrees, rate is degrees/S.
//Assuming that Step is called at a fixed period (uS)
class M3TimeSlew
{
	public:
		M3TimeSlew():val(0)
		{
			dt = (mReal)RT_TIMER_TICKS_NS/1000000000.0; //Seconds
		}
		mReal Step(mReal des, mReal rate)
		{			
			if (val<des)
				val=MIN(des,val+dt*rate);
			else
				val=MAX(des,val-dt*rate);
			return val;
		}
		void Reset(mReal v){val=v;}
	private:
		mReal  dt;
		mReal val;
};


//Get the value change over a time window (uS)
//Assuming that Step is called with a fixed period (uS)
class M3TimeDerivative
{
public:
	M3TimeDerivative(int window_us)
	{
		int period_us = RT_TIMER_TICKS_NS/1000;
		
		int n=(window_us/period_us)+1;
		if (n<3)
		{
			M3_INFO("M3TimeDerivative window too small: %d %d %d\n",n,window_us, period_us);
			return;
		}
		val.resize(n,0);
		timestamp.resize(n,0);
		idx_low=0;
		idx_high=n-1;
	}
	
	mReal Step(mReal v, int64_t ts)
	{
		if (val.size())
		{
			val[idx_high]=v;
			timestamp[idx_high]=ts;
			mReal dt=(mReal)(timestamp[idx_high]-timestamp[idx_low]);
			mReal dv=val[idx_high]-val[idx_low];
			idx_low=(idx_low+1)%(int)val.size();
			idx_high=(idx_high+1)%(int)val.size();
			if (dt==0) //startup
				return 0;
			return dv/dt;
		}
		return 0;
	}
	private:
		vector<mReal> val;
		vector<int64_t> timestamp;
		int idx_low;
		int idx_high;
};

class M3DitherToInt
{
	public:
		int Step(mReal val)
		{	
			mReal p= (mReal)rand()/(mReal)RAND_MAX; //0-1.0
			mReal rem=val-floor(val);
			if (rem==0) return (int)val;
			if (p<rem) return  (int)ceil(val); //round up
			if (p>rem) return  (int)floor(val);
			return val;
		} 
};

mReal EvalCalibrationPoly(vector<mReal> & p, mReal x);


class M3PID
	{
		public:
			M3PID():i_error_sum(0),tmp_cnt(0){};
			mReal Step(mReal sense, mReal sense_dot,mReal des, mReal p, mReal i, 
						mReal d, mReal i_limit, mReal i_range);
			void ResetIntegrator();
		private:
			mReal i_error_sum;
			int tmp_cnt;
			mReal p_term,d_term,i_term,e;

	};
	

//Measure the peak-to-peak excursion of a cyclic signal about 0
class M3PeakToPeak
{
  public:
	M3PeakToPeak():x_last(0),first(1),x_min(0),x_max(0),x_min_tmp(0),x_max_tmp(0),n_cyc(0){};
	
	void Step(mReal x,mReal x_zero)
	{
	  if (first)
	  {
	    if (x>x_zero)
	      x_last=INT_MIN;
	    else
	      x_last=INT_MAX;
	    first=0;
	  }
	  if (x>x_zero and x_last<=x_zero) //Rising transition
	  {
	    x_min=x_min_tmp;
	    x_min_tmp=INT_MAX;//something big
	    n_cyc++;
	  }
	  
	  if (x<=x_zero and x_last>x_zero) //Falling transition
	  {
	    x_max=x_max_tmp;
	    x_max_tmp=INT_MIN;//something small
	  }
	  if (x>x_zero)
	    x_max_tmp=MAX(x_max_tmp,x);
	  if (x<=x_zero) 
	    x_min_tmp=MIN(x_min_tmp,x);
	  x_last=x;
	}
	mReal GetMin(){return x_min;}
	mReal GetMax(){return x_max;}
	mReal GetP2P(){return x_max-x_min;}
	mReal GetCycles(){return n_cyc;}
  private:
    mReal x_last;
    mReal x_min,x_min_tmp;
    mReal x_max,x_max_tmp;
    mReal n_cyc;
    int first;
};

}//namespace


















#endif

