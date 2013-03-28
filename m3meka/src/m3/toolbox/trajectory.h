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

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "m3/toolbox/trajectory.pb.h"
#include "m3/toolbox/toolbox.h"


namespace m3
{
	using namespace std;
	using namespace KDL;


//Handle smooth joint trajectories using cubic splines.
//See Craig, Intro to Robotics, Joint space schemes
//New vias can be added dynamically.
	

//From http://www.shadmehrlab.org/book/minimumjerk.pdf 
//Note: error in text for a2 term. Should be D**2
class M3MinJerkTrajectory
{
	public:
		M3MinJerkTrajectory():first(1),x(0),v(0),p(0),j(0){}
		void Startup(int64_t timestamp, mReal theta);
		mReal Step (int64_t timestamp, mReal theta_des, mReal thetadot_des);
		mReal GetTheta(){return x;}
		mReal GetThetaDot(){return v;}
		mReal GetThetaDotDot(){return p;}
		mReal GetThetaDotDotDot(){return j;}
	private:
		void Compute(mReal t);
		void Create(mReal mxi,mReal mvi,mReal mpi,mReal mxf,mReal mt0,mReal mtf);
		mReal t0,tf,xf,D,a0,a1,a2,a3,a4,a5;
		mReal x,v,p,j;
		int64_t tbase;
		int first;
		int tmp_cnt;
};

//Handle smooth joint trajectories using cubic splines.
//See Craig, Intro to Robotics, Joint space schemes
//New vias can be added dynamically.
	
class JointSplineSegment
{
	public:
		JointSplineSegment(JntArray & q_0, JntArray & q_f, 
				   JntArray & qdot_0,JntArray & qdot_f,mReal tf)
		{
			int nq = q_0.rows();
			a0.resize(nq);
			a1.resize(nq);
			a2.resize(nq);
			a3.resize(nq);
			for (int i=0; i<nq; i++)
			{
				a0(i)=q_0(i);
				a1(i)=qdot_0(i);
				a2(i)=(3/(pow(tf,2)))*(q_f(i)-q_0(i))-(2/tf) * qdot_0(i)-(1/tf)*qdot_f(i);
				a3(i)=(-2/(pow(tf,3)))*(q_f(i)-q_0(i))+(1/pow(tf,2))*(qdot_f(i)+qdot_0(i));
			}
			
			//a2=(3/ (pow(tf,2)))*(q_f-q_0)-(2/tf)  * qdot_0-(1/tf)*qdot_f;
			//a3=(-2/(pow(tf,3)))*(q_f-q_0)+(1/pow(tf,2))*(qdot_f+qdot_0);
						
			duration=tf;
		}
		mReal GetTimeElapsed(){return elapsed;}
		mReal GetTimeDuration(){return duration;}
		bool Step(mReal t, JntArray & q)
		{
			int nq = q.rows();
			elapsed=t;
			if (t<=duration)
			{
				for (int i=0; i<nq; i++)
					q(i) = a0(i)+a1(i)*t+a2(i)*pow(t,2)+a3(i)*pow(t,3);
				return true;
			}
			for (int i=0; i<nq; i++)
				q(i) = a0(i)+a1(i)*duration+a2(i)*pow(duration,2)+a3(i)*pow(duration,3);
			
			return false;
		}
	private:
		JntArray a0,a1,a2,a3;
		mReal duration;
		mReal elapsed;
};


class M3JointTrajectory
{
	public:
		M3JointTrajectory();
		~M3JointTrajectory();
	public:
		void Reset(vector<bool> active,int num_dof);
		void AddVia(const M3JointVia & v);
		int Step(int64_t timestamp, JntArray & theta_deg, JntArray & thetadot_deg, JntArray & q_des_deg);
	private:
		void AddSplines(JntArray & theta, JntArray & thetadot);
		mReal GetDuration(JntArray & qstart, JntArray & q_end, JntArray & qd_avg);
		JntArray q_0,q_f,qdot_0,qdot_f,qdot_avg, qn_0,qn_f,qndot_avg,qndot_0,q_des_last;
		int ndof;
		vector<JointSplineSegment *> splines;
		vector<M3JointVia> vias;
		int vidx_last;
		vector<bool> active_dof;
		vector<int> spline_idx;
		int nactive;
		bool set_idle_q;
		bool reset;
		mReal ts_start;
		JointSplineSegment * current;
		int completed_spline_idx;
		int tmp_cnt;
		JntArray theta_rad, thetadot_rad, q_des_rad;
		M3JointVia new_via;
};

}


#endif


