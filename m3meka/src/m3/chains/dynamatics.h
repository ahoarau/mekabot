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

#ifndef M3_DYNAMATICS_H
#define M3_DYNAMATICS_H

#include "m3rt/base/component.h"
#include "m3/chains/dynamatics.pb.h"
#include "m3/toolbox/toolbox.h"
#include <google/protobuf/message.h>
#include "m3/chains/joint_chain.h"


namespace m3
{
	using namespace std;
	using namespace KDL;

///////////////////////////////////////////////////////////////////////////
class M3JointChain;

class M3Dynamatics : public m3rt::M3Component
{
	public:
		M3Dynamatics() : M3Component(DYNAMATICS_PRIORITY), fksolver_vel(NULL), idsolver(NULL), jjsolver(NULL) 
		{
			RegisterVersion("default",DEFAULT);	//RBL
			RegisterVersion("iss",ISS);		//ISS. Safe as DEFAULT
			//RegisterVersion("iq",IQ);
		}
		~M3Dynamatics()
		{
			if (fksolver_vel!=NULL) delete fksolver_vel;fksolver_vel=NULL;
			if (idsolver!=NULL) delete idsolver;idsolver=NULL;			
			if (jjsolver!=NULL) delete jjsolver;jjsolver=NULL;			
		}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
		int GetNumDof(){return ndof;}
		Vector GetEndPos(){return end_pos;}
		Rotation GetEndRot(){return end_rot;}
		Twist GetEndTwist(){return end_twist;}
		mReal GetG(int i){return G(i+3);}		
		Vector GetPayloadCom(){return payload_com;}
		mReal GetPayloadMass(){return param.payload_mass();}		
		void SetGrav(Vector new_grav){grav = new_grav;}
		Wrench GetBaseWrench(){return base_wrench;}
		void SetEndWrench(Wrench wrench){end_wrench = wrench;}
		mReal GetMass(){return chain_mass+GetPayloadMass();}
		int GetNumSegments(){return kdlchain.getNrOfSegments();}
		RotationalInertia GetPayloadInertia(){return payload_I;}
	private:
		enum {DEFAULT, ISS};	
		M3JointChain * m3chain;
		string chain_name;
		Rotation end_rot;				
		FrameVel end_2_base_framevel;
		 // Using Inverse Jacobian should be avoided in favor of faster numerical solvers...
		Jacobian J; //6x(ndof) Jacobian
		JntArray G; //gravity , (ndof)x1
		JntArray qdotdot_id;
		JntArray qdot_id;
		JntArray zero_vec;
		Twist end_twist;
		Vector end_pos;
		Chain kdlchain;		
		JntArrayVel q; // both q and qdot 
		// for end frame with no load
		Vector z_com;
		RotationalInertia z_I;
		mReal z_m;
		Vector grav;
		ChainIdSolver_RNE * idsolver;
		std::vector<Wrench> f_ext;
		ChainFkSolverVel_recursive * fksolver_vel;
		//ChainFkSolverPos_recursive * fksolver_pos;
		ChainJntToJacSolver * jjsolver;
		int ndof;
		Vector payload_com;
		Wrench end_wrench;
		Wrench base_wrench;
		mReal chain_mass;
		vector<mReal> d;
		vector<mReal> a;
		vector<mReal> alpha;
		vector<mReal> joint_offset;
		vector<mReal> m;
		vector<mReal> cx;
		vector<mReal> cy;
		vector<mReal> cz;
		vector<mReal> Ixx;
		vector<mReal> Ixy;
		vector<mReal> Ixz;
		vector<mReal> Iyy;
		vector<mReal> Iyz;
		vector<mReal> Izz;
		RotationalInertia payload_I;
		bool use_velocities;
		bool use_accelerations;
	protected:
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();			
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
		M3DynamaticsStatus status;
		M3DynamaticsCommand command;
		M3DynamaticsParam param;
		void SetPayload();
		int tmp_cnt;
};

}

#endif


