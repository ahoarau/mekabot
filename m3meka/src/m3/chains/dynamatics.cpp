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

#include "m3/chains/dynamatics.h"


namespace m3
{
	
using namespace m3rt;
using namespace std;
using namespace KDL;

bool M3Dynamatics::LinkDependentComponents()
{	
	m3chain = (M3JointChain*)factory->GetComponent(chain_name);
	
	if (m3chain!=NULL)
		return true;
	else
	{
		M3_ERR("M3Dynamatics: no chain found with name: %s\n", chain_name.c_str());
	}
	return false;
}

void M3Dynamatics::Startup()
{		
	if (ndof != m3chain->GetNumDof())
	{
		M3_WARN("M3Dynamatics: ndof does not match chain: %s\n", chain_name.c_str());		
	}
	
	q.resize(ndof+3);	// contains both position q.q and vel q.qdot
	qdot_id.resize(ndof+3);
	qdotdot_id.resize(ndof+3);		
	G.resize(ndof+3);
	J=Jacobian(ndof+3);	
	chain_mass=0;
	for (int i=0; i<ndof; i++)
	{
		status.add_g(0);
		chain_mass += m[i];
	}

	for (int i=0; i<3; i++)
	{
		status.add_end_pos(0);
	}

	for (int i=0; i<9; i++)
	{
		status.add_end_rot(0);
	}

	for (int i=0; i<6; i++)
	{
		status.add_end_twist(0);
		status.add_base_wrench(0);
		status.add_child_wrench(0);
	}

	for (int i=0; i<(6*ndof); i++)
	{
		status.add_j(0);
	}
	
	for (int i=0; i<=ndof; i++)
	{		
		// KDL adds each joint to the BASE of each frame, not the end as Roboop did
		Frame frame = Frame().DH_Craig1989(a[i], DEG2RAD(alpha[i]), d[i], DEG2RAD(joint_offset[i]));
		
		// The first segment is static defining the offset from the origin,
		// but we are adding a joint in each direction that will always be zero to calc
		// wrenches at the base
		
		if (i==0) 
		{
			kdlchain.addSegment(Segment(Joint(Joint::RotX)));
			kdlchain.addSegment(Segment(Joint(Joint::RotY)));
			kdlchain.addSegment(Segment(Joint(Joint::RotZ), frame));
		} else {			
			// because frames define the joint placement for the NEXT segment, we need to use Inertial values for previous segment
			Vector vcog = Vector(cx[i-1] ,cy[i-1], cz[i-1]);					
			RotationalInertia ri = RotationalInertia(Ixx[i-1], Iyy[i-1], Izz[i-1], Ixy[i-1], Ixz[i-1], Iyz[i-1]);
			kdlchain.addSegment(Segment(Joint(Joint::RotZ), frame, frame.Inverse()*RigidBodyInertia(m[i-1],vcog, ri)));
		}
	}
	
	//Store no-load payload for last link
	Frame toTip = kdlchain.getSegment(kdlchain.getNrOfSegments()-1).getFrameToTip();
	RigidBodyInertia z_I_rigid = toTip*kdlchain.getSegment(kdlchain.getNrOfSegments()-1).getInertia();
	z_com = z_I_rigid.getCOG();
	z_I = z_I_rigid.getRotationalInertia();
	z_m = z_I_rigid.getMass();
	
	fksolver_vel = new ChainFkSolverVel_recursive(kdlchain);
	grav = Vector(0,0,1.)*GRAV;
	idsolver = new ChainIdSolver_RNE(kdlchain, grav);
	f_ext = std::vector<Wrench>(kdlchain.getNrOfSegments());
	jjsolver = new ChainJntToJacSolver(kdlchain); 

	SetPayload();
	SetStateSafeOp();
}



void M3Dynamatics::Shutdown()
{

}

void M3Dynamatics::StepCommand()
{
	
}

void M3Dynamatics::StepStatus()
{	
	if (IsStateError())
		return;
	// ToDo: find out why copy operator= causes lockups in hard RT	
	for (int i=0; i<3; i++)
	{
		q.q(i) = 0.;
		q.qdot(i) = 0.;
		qdotdot_id(i) = 0.;
		qdot_id(i) = 0.;
	}	
	for (int i=0; i<ndof; i++)
	{
		q.q(i+3) = m3chain->GetThetaRad(i);
		q.qdot(i+3) = m3chain->GetThetaDotRad(i);
		if (param.use_accelerations())
		{
			qdotdot_id(i+3) = m3chain->GetThetaDotDotRad(i);
		}
		else
			qdotdot_id(i+3) = 0.;
		if (param.use_velocities())
			qdot_id(i+3) = m3chain->GetThetaDotRad(i);
		else
			qdot_id(i+3) = 0.;

	}	
	SetPayload();	
	idsolver->SetGrav(grav);
	
	f_ext[kdlchain.getNrOfSegments()-1] = end_wrench;	
	
	int result = idsolver->CartToJnt(q.q, qdot_id, qdotdot_id, f_ext, G);
	
	for (int i=0; i<G.rows(); i++)
		G(i) = -1000*G(i);  // Nm to mNm
	if (result==-1)
		M3_ERR("ID solver returned error %d for M3Kinestatics component %s\n", result, GetName().c_str());
	jjsolver->JntToJac(q.q, J);
	fksolver_vel->JntToCart(q, end_2_base_framevel);
	//fksolver_pos->JntToCart(q.q,T80);
	end_twist = Twist(end_2_base_framevel.p.v, end_2_base_framevel.M.w);
	end_pos = end_2_base_framevel.p.p;
	end_rot = end_2_base_framevel.M.R;
	base_wrench.force = grav*GetMass();
	base_wrench.torque = Vector(G(0),G(1),G(2))/1000.;
	
	for (int i=0; i<ndof; i++){
		status.set_g(i, G(i+3));
	}
	
	for (int i=0; i<3; i++)
		status.set_end_pos(i, end_pos(i));

	for (int i=0; i<6; i++)
	{
		if (i<3)
		{
			status.set_end_twist(i, end_twist.vel(i));
			status.set_base_wrench(i, base_wrench.force(i));
			status.set_child_wrench(i, end_wrench.force(i));
		} else {
			status.set_end_twist(i, RAD2DEG(end_twist.rot(i-3)));
			status.set_base_wrench(i, base_wrench.torque(i-3));
			status.set_child_wrench(i, end_wrench.torque(i-3));
		}
	}
	for (int i=0; i<9; i++)
		status.set_end_rot(i, end_rot.data[i]);

	for (int i=0; i<6; i++)
	{
		for (int j=0; j<ndof; j++)
			status.set_j((i*ndof)+(j), J(i,j+3));
	}

	for (int i=0; i<ndof; i++)
		m3chain->SetG(i, GetG(i));
	m3chain->SetGCoupled();	
}

// This should be realtime safe... Explanation:
// When first created in StartUp idsolver creates a vector of segments for the chain allocating necessary memory.
// When the constructor is called again it resizes the vector to zero, however when it rebuilds it it should
// not exceed the size of the previous allocation and not reallocate any memory.
void M3Dynamatics::SetPayload()
{
	if (param.payload_inertia_size()==6)
	{		
		//param.payload_inertia defined as Ixx, Ixy, Ixz, Iyy, Iyz, Izz
		// KDL defines RotationalInertia(Ixx,Iyy,Izz,Ixy,Ixz,Iyz);
		payload_I = RotationalInertia(param.payload_inertia(0),
						param.payload_inertia(3),
						param.payload_inertia(5),
						param.payload_inertia(1),
						param.payload_inertia(2),
						param.payload_inertia(4));
	}
	else
	{
		//Possible bug, size changes from 0-3. Likely need of mutex. Appears to be fixed. Keep this just in case.
		M3_ERR("Bug: Bad field size in M3Dynamatics::payload_inertia...%d instead of  6\n", param.payload_inertia_size());
		PrettyPrint();
	}
	if (param.payload_com_size()==3)
	{
		for (int i; i<3; i++)
			payload_com(i)=param.payload_com(i);		
	}
	else
	{
		//Possible bug, size changes from 0-3. Likely need of mutex. Appears to be fixed. Keep this just in case.
		M3_ERR("Bug: Bad field size in M3Dynamatics::payload_com...%d instead of 3 \n", param.payload_com_size());
		PrettyPrint();
	}

	Frame toTip = kdlchain.getSegment(kdlchain.getNrOfSegments()-1).getFrameToTip();
	Segment * end_eff = kdlchain.getMutableSegment(kdlchain.getNrOfSegments()-1);
	mReal m = GetPayloadMass();
	Vector com = toTip*GetPayloadCom(); // tranforming from wrist to last joint's frame where we defined it's COM...
	RotationalInertia rot_inertia = (toTip*RigidBodyInertia(0.,Vector(0.,0.,0.),GetPayloadInertia())).getRotationalInertia();
						
	if (m+z_m>0.001)
	{
		Vector ecom = (m*com+z_com*z_m)/(m+z_m);		
		end_eff->setInertia(toTip.Inverse()*RigidBodyInertia(m+z_m, ecom, rot_inertia + z_I));
	}
	else
	{
		Vector ecom = (com+z_com);		
		end_eff->setInertia(toTip.Inverse()*RigidBodyInertia(m+z_m, ecom, rot_inertia + z_I));
	}
		
	idsolver->chain = kdlchain;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3Dynamatics::ReadConfig(const char * filename)
{
	if (!M3Component::ReadConfig(filename))
		return false;
	
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	
	mReal rtemp;
	int itemp;
	
	doc["chain_component"] >> chain_name;	
	doc["ndof"] >> ndof;	
	
	const YAML::Node& ymlparam = doc["param"];

	ymlparam["payload_mass"] >> rtemp;
	param.set_payload_mass(rtemp);	
	for(int i=0; i<ymlparam["payload_com"].size(); i++)
	{
		ymlparam["payload_com"][i] >> rtemp;
		param.add_payload_com(rtemp);
		payload_com(i)=rtemp;
	}

	for(int i=0; i<ymlparam["payload_inertia"].size(); i++)
	{
		ymlparam["payload_inertia"][i] >> rtemp;
		param.add_payload_inertia(rtemp);
	}

	payload_I = RotationalInertia(param.payload_inertia(0),
						param.payload_inertia(3),
						param.payload_inertia(5),
						param.payload_inertia(1),
						param.payload_inertia(2),
						param.payload_inertia(4));

	bool btemp;
	try
	{
	    ymlparam["use_velocities"] >> btemp;
	    param.set_use_velocities(btemp);
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		param.set_use_velocities(0);
	} 
	try
	{
	  ymlparam["use_accelerations"] >> btemp;
	  param.set_use_accelerations(btemp);
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		param.set_use_accelerations(0);
	} 	


	const YAML::Node& links = doc["links"];

	for(int i=0; i<links.size(); i++)
	{		
		links[i]["d"] >> rtemp;
		d.push_back(rtemp);
		links[i]["a"] >> rtemp;
		a.push_back(rtemp);
		links[i]["alpha"] >> rtemp;
		alpha.push_back(rtemp);
		links[i]["joint_offset"] >> rtemp;
		joint_offset.push_back(rtemp);
		if (i < (links.size()-1))
		{
			links[i]["m"] >> rtemp;
			m.push_back(rtemp);
			links[i]["cx"] >> rtemp;
			cx.push_back(rtemp);
			links[i]["cy"] >> rtemp;
			cy.push_back(rtemp);
			links[i]["cz"] >> rtemp;
			cz.push_back(rtemp);
			if(const YAML::Node *pName = links[i].FindValue("Ixx")) {
    				*pName >> rtemp;
			} else {
    				rtemp = 0.0;
			}
			Ixx.push_back(rtemp);
			if(const YAML::Node *pName = links[i].FindValue("Ixy")) {
    				*pName >> rtemp;
			} else {
    				rtemp = 0.0;
			}
			Ixy.push_back(rtemp);
			if(const YAML::Node *pName = links[i].FindValue("Ixz")) {
    				*pName >> rtemp;
			} else {
    				rtemp = 0.0;
			}
			Ixz.push_back(rtemp);
			if(const YAML::Node *pName = links[i].FindValue("Iyy")) {
    				*pName >> rtemp;
			} else {
    				rtemp = 0.0;
			}
			Iyy.push_back(rtemp);
			if(const YAML::Node *pName = links[i].FindValue("Iyz")) {
    				*pName >> rtemp;
			} else {
    				rtemp = 0.0;
			}
			Iyz.push_back(rtemp);
			if(const YAML::Node *pName = links[i].FindValue("Izz")) {
    				*pName >> rtemp;
			} else {
    				rtemp = 0.0;
			}
			Izz.push_back(rtemp);
		}
	}

	return true;
}

}