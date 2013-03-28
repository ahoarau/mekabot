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

#include "trajectory.h"

namespace m3
{
	
using namespace std;
using namespace KDL;


void M3MinJerkTrajectory::Create(mReal mxi,mReal mvi,mReal mpi,mReal mxf,mReal mt0,mReal mtf)
{
    t0=mt0;
	tf=mtf;
	xf=mxf;
	D=mtf-mt0;
	a0=mxi;
	a1=D*mvi;
	a2=pow(D,2)*mpi/2.0;
	a3=-1.5*mpi*pow(D,2) - 6*mvi*D + 10*(mxf-mxi);
	a4= 1.5*mpi*pow(D,2) + 8*mvi*D - 15*(mxf-mxi);
	a5=-0.5*mpi*pow(D,2) - 3*mvi*D +  6*(mxf-mxi);
	/*M3_INFO("Create t0 %f\n",t0);
	M3_INFO("Create tf %f\n",tf);
	M3_INFO("Create xf %f\n",mxf);
	M3_INFO("Create xi %f\n",mxi);
	M3_INFO("Create vi %f\n",mvi);
	M3_INFO("Create pi %f\n",mpi);
	M3_INFO("Create a1 %f\n",a1);
	M3_INFO("Create a2 %f\n",a2);
	M3_INFO("Create a3 %f\n",a3);
	M3_INFO("Create a4 %f\n",a4);
	M3_INFO("Create a5 %f\n",a5);*/
}

void M3MinJerkTrajectory::Startup(int64_t timestamp, mReal theta )
{
	tbase=timestamp;
	x=theta;
	v=0; //Since entering from off-state, assume these are 0
	p=0;
	j=0.0;
	first=1;
}

void M3MinJerkTrajectory::Compute(mReal t)
{

	mReal tau=(t-t0)/D;
	
	if (tau>1.0)
	{
		x=xf;v=0.0;p=0.0;j=0.0;
		return;
	}
	x=	a0 + 
		a1*tau +
		a2*pow(tau,2)+
		a3*pow(tau,3)+
		a4*pow(tau,4)+
		a5*pow(tau,5);
	/*if (tmp_cnt++%100==0)
		M3_INFO("T: %f Tau: %f ",t,tau);
	if (tmp_cnt%100==0)	
		M3_INFO("X %f\n",x);*/
	
	v=	a1/D +
		2*a2*tau/D+
		3*a3*pow(tau,2)/D+
		4*a4*pow(tau,3)/D+
		5*a5*pow(tau,4)/D;
		
	p=	2*a2/pow(D,2)+
		6*a3*tau/pow(D,2)+
		12*a4*pow(tau,2)/pow(D,2)+
		20*a5*pow(tau,3)/pow(D,2);
	
	j=	6*a3/pow(D,3)+
		24*a4*tau/pow(D,3)+
		60*a5*pow(tau,2)/pow(D,3);
}

mReal M3MinJerkTrajectory::Step (int64_t timestamp, mReal theta_des, mReal thetadot_des)
{
	mReal t = (timestamp-tbase)/1000000.0;
	if (!first)
		Compute(t);
	if (xf!=theta_des || first)
	{
		first=0;
		mReal mtf=t+ABS(theta_des-x)/MAX(.001,ABS(thetadot_des));
		Create(x,v,p,theta_des,t,mtf);
	}
	return x;
}

M3JointTrajectory::~M3JointTrajectory()
{
	for (int i=0;i<splines.size();i++)
		delete splines[i];
}
M3JointTrajectory::M3JointTrajectory()
{
	vidx_last=-1;
	ndof=0;
	nactive=0;
	set_idle_q=false;
	current=NULL;
	reset=true;	
}
void M3JointTrajectory::Reset(vector<bool> active,int num_dof)
{
	vidx_last=-1;
	//M3_INFO("M3JointTrajectory::Reset %d splines active \n",splines.size());
	for (int i=0;i<splines.size();i++)
	{
		delete splines[i];
	}
	splines.clear();
	spline_idx.clear();
	vias.clear();
	ndof=num_dof;
	active_dof=active;
	q_0=JntArray(ndof);
	q_f=JntArray(ndof);
	qdot_0=JntArray(ndof);
	qdot_f=JntArray(ndof);
	qdot_avg=JntArray(ndof);
	qn_0=JntArray(ndof);
	qn_f=JntArray(ndof);
	qndot_0=JntArray(ndof);
	qndot_avg=JntArray(ndof);
	theta_rad=JntArray(ndof);
	thetadot_rad=JntArray(ndof);
	q_des_rad=JntArray(ndof);
	q_des_last=JntArray(ndof);
	nactive=0;
	set_idle_q=true;
	reset=true;
	completed_spline_idx=-1;
	for (int i=0;i<ndof;i++)
	{
		if (active[i])
			nactive++;
		new_via.add_q_desired(0);
		new_via.add_qdot_avg(0);
		q_0(i)=0.;
		q_f(i)=0.;
		qdot_0(i)=0.;
		qdot_f(i)=0.;
		qdot_avg(i)=0.;
		qn_0(i)=0.;
		qn_f(i)=0.;
		qndot_0(i)=0.;
		qndot_avg(i)=0.;
		theta_rad(i)=0.;
		thetadot_rad(i)=0.;
		q_des_rad(i)=0.;
		q_des_last(i)=0.;
	}
	current=NULL;
}

void M3JointTrajectory::AddVia(const M3JointVia & v)
{
	if (v.idx()<=vidx_last) //Duplicate
		return;
	vidx_last=v.idx();
	if (nactive)
	{
		new_via.set_idx(v.idx());
		for (int i=0; i<ndof; i++)
		{
			new_via.set_q_desired(i,DEG2RAD(v.q_desired(i)));
			
			new_via.set_qdot_avg(i,DEG2RAD(v.qdot_avg(i)));
		}
//M3_INFO("Added via %d: J0 %f\n",v.idx(),v.q_desired(0));
		vias.push_back(new_via);
	}
}
		
//Compute spline duration as the time it takes to move the greatest joint displacement at the avg velocity
//In practice, actual velocity may vary
mReal M3JointTrajectory::GetDuration(JntArray & qstart, JntArray & q_end, JntArray & qd_avg)
{
	mReal duration=0;
	for (int i=0;i<ndof;i++)
	{
		if (qd_avg(i)>0)
		{
			mReal d=ABS((q_end(i)-qstart(i))/qd_avg(i));
			if (d>duration&&active_dof[i])
				duration=d;
		}
	}
	return duration;
}


void M3JointTrajectory::AddSplines(JntArray & theta, JntArray & thetadot)
{
	mReal tf_0, tf_1,m0,m1;
	
	//M3_INFO("Adding new Spline segment (multi)\n",0);
	
	
	int nv=vias.size();
	if (nv==0)
		return;

	//M3_INFO("Adding new Spline segment (multi): Idx %d Duration: %f\n",vias[0].idx(),tf_0);
	
	for (int j=0;j<vias.size();j++)
	{
		if (j==0) //first
		{
			if (splines.size()) //Adding to an existing spline, use the previous q_f (state is held unless reset)
			{
				for (int i=0;i<ndof;i++)
				{	
					q_0(i)=q_f(i);
					qdot_0(i)=qdot_f(i);
				}
			}
			else //No existing spline, use the current posture to start
			{
				for (int i=0;i<ndof;i++)
				{	
					q_0(i)=theta(i);
					qdot_0(i)=thetadot(i);
				}
			}
		}
		for (int i=0;i<ndof;i++)
		{
			if (j!=0)
			{
				q_0(i)=vias[j-1].q_desired(i);
				qdot_0(i)=qndot_0(i);
			}
			q_f(i)=vias[j].q_desired(i);
			qdot_f(i)=0; //Default
			qdot_avg(i)=vias[j].qdot_avg(i);
			//Next segment
			if (j+1<vias.size())
			{
				qn_0(i)=q_f(i);
				qn_f(i)=vias[j+1].q_desired(i);
				qndot_avg(i)=vias[j+1].qdot_avg(i);
			}
		}
		
		tf_0=MAX(.001,GetDuration(q_0,q_f,qdot_avg));//current duration
		
		//Overwrite final velocity if a next via
		if (j+1<vias.size())
		{
			tf_1=MAX(.001,GetDuration(qn_0,qn_f,qndot_avg)); //next duration
			for (int i=0;i<ndof;i++)
			{
				m0=(q_f(i)-q_0(i))/tf_0;	//slope of current segment (rad/S)
				m1=(qn_f(i)-qn_0(i))/tf_1;	//slope of next segment (rad/S)
				if ((m0>0&&m1<0)||(m0<0&&m1>0)) //slope changes, zero velocity point
				{
					qdot_f(i)=0;
					qndot_0(i)=0;
				}
				else
				{
					qdot_f(i)=(m0+m1)/2;	
					qndot_0(i)=(m0+m1)/2;	
				}
			}
		}
		splines.push_back(new JointSplineSegment(q_0,q_f,qdot_0,qdot_f,tf_0));
		spline_idx.push_back(vias[j].idx());
		//M3_INFO("Adding new Spline segment (single): Idx %d Duration: %f\n",vias[j].idx(),tf_0);
	}
	vias.clear();
}



int M3JointTrajectory::Step(int64_t timestamp, JntArray & theta_deg, JntArray & thetadot_deg, JntArray & q_des_deg)
{
	for(int i=0; i<ndof; i++)
	{
		theta_rad(i) = DEG2RAD(theta_deg(i));
		thetadot_rad(i) = DEG2RAD(thetadot_deg(i));
	}
	mReal ts=timestamp/1000000.0; //Do this all in seconds, radians
	AddSplines(theta_rad,thetadot_rad);
	if (splines.size())
	{
		if (splines[0] !=current)
			ts_start=ts;
		current=splines[0];
		if (!splines[0]->Step(ts-ts_start, q_des_rad))
		{			
			completed_spline_idx=spline_idx[0];
			set_idle_q=true;			
			delete splines[0];			
			splines.erase(splines.begin());			
			spline_idx.erase(spline_idx.begin());			
			for(int i=0; i<ndof; i++)			
				q_des_last(i)=q_des_rad(i);						
			current=NULL;			
		}	
	}
	else
	{
		if (set_idle_q) //choose setpoint as last command
		{
			if (reset)
			{
				reset=false;
				for(int i=0; i<ndof; i++)				
					q_des_rad(i)=theta_rad(i);
				
			}
			else
			{
				for(int i=0; i<ndof; i++)	
					q_des_rad(i)=q_des_last(i);
			}
			set_idle_q=false;
		}
	}
	for(int i=0; i<ndof; i++)	
		q_des_deg(i) = RAD2DEG(q_des_rad(i));		
	
	return completed_spline_idx;
}



}