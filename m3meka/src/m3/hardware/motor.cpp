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

#include "m3/hardware/motor.h"
#include "math.h"
#include <cmath>
#include <Eigen/LU>
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace m3{
	
using namespace std;


void M3MotorModel::ThermalInit(string config_filename)
{

	mReal C1, C2, tau1, tau2, tau3;

	//Initialize matrices
	eA2 = VectorXf::Zero(2);	
	Tprev = VectorXf::Zero(2);	
	eA1 = MatrixXf::Zero(2,2);
	
	tau1 = thermal_time_constant_winding;
	tau2 = thermal_time_constant_motor;
	C1 = tau1/thermal_resistance_rotor_housing;
	C2 = tau2/thermal_resistance_housing_ambient;
	tau3 =  thermal_resistance_rotor_housing*C2;
	
	MatrixXf I=MatrixXf::Zero(2,2);
	MatrixXf ser=MatrixXf::Zero(2,2);
	MatrixXf A=MatrixXf::Zero(2,2);
	VectorXf B=VectorXf::Zero(2);
	
	mReal dt = 1./RT_TASK_FREQUENCY;
	I << 1,0,0,1;
	
//	eA1 = exp(-1/tau1/RT_TASK_FREQUENCY);
//	eA2 = -tau1*(eA1-1)*1/C1;
	
	A << -1/tau1, 1/tau1,
	      1/tau3, -1/tau3-1/tau2;
	B << 1/C1, 0;
	
	
	ser = I;
	eA1 = I;
	for (int i = 1;i<5;i++) {
	    ser = ser*A*dt/i;
	    eA1 = eA1 + ser;
	}
	eA2 = A.inverse()*(eA1-I)*B;
	
		
	Tprev << 0.,0.;
	
	if (use_old_temp_values)
	{
	  
	    // Get Old Temps to init model
	    string temp_filename = config_filename.substr(0, config_filename.length()-4);
	    temp_filename += "_temps.yml";
	    YAML::Node doc;
	    
	    bool previous_temp_missing = false;	
	    
	    
	    string s(temp_filename);
	    string path;
	    
	    if (GetEnvironmentVar(M3_ROBOT_ENV_VAR, path))
	    {		
		    path.append("/robot_config/");
		    path.append(s);
	    }
	    
	    ifstream fin(path.c_str());
	    if (fin.fail())
	    {		
		    //M3_ERR("could not read %s \n", path.c_str());	
		    return;
	    }

	    YAML::Parser parser(fin);
	    
	    parser.GetNextDocument(doc);
	    fin.close();
	    
	    /*try
	    {
	      GetYamlDoc(temp_filename.c_str(), doc);
	    } catch(YAML::RepresentationException e) {
	      previous_temp_missing = true;
	      M3_DEBUG("b\n");
	    }
	    catch(...) {
	      previous_temp_missing = true;
	      M3_DEBUG("a\n");
	    }*/	
	    
	    if (!previous_temp_missing)
	    {
	      try 
	      {
		
		      doc["previous_winding_temp"] >> previous_winding_temp;
		      
	      } catch(YAML::TypedKeyNotFound<string> e) 
	      {		
		      previous_winding_temp=0.0;
	      } 

	      try 
	      {
		      doc["previous_case_temp"] >> previous_case_temp;
		      
	      } catch(YAML::TypedKeyNotFound<string> e) 
	      {		
		      previous_case_temp=0.0;
	      } 


	      try 
	      {
		      doc["previous_temp_timestamp"] >> previous_temp_timestamp;
		      
	      } catch(YAML::TypedKeyNotFound<string> e) 
	      {
		      //M3_WARN("Missing version key in config file for motor %s. Defaulting to type MODEL_V0\n",name.c_str());
		      previous_temp_timestamp="";
	      } 
	      
	      if (previous_temp_timestamp != "")
	      {
			time_t timenew;		    		    
			time(&timenew);		    
					
			struct tm timeold;		  		  		    
			if (strptime(previous_temp_timestamp.c_str(), "%c", &timeold) == NULL) //ERROR
			  return;		    
			time_t oldtime = mktime(&timeold);
			
			double diff = difftime(timenew, oldtime);
			if (diff > 2.0*thermal_time_constant_motor)
				return;

			Tprev << previous_winding_temp, previous_case_temp;
			int model_run_cnts = int(diff / dt);
			for (int i = 0; i < model_run_cnts; i++)
			{
				Tprev = eA1*Tprev;
			}

			/*ser = I;
			MatrixXf eA1_i = I;
			for (int i = 1;i<5;i++) {
			    ser = ser*A*diff/i;
			    eA1_i = eA1_i + ser;
			}
			Tprev << previous_winding_temp, previous_case_temp;
			Tprev = eA1_i*Tprev;
			if (Tprev[0] < 0.0)
				Tprev[0] = 0.0;
			if (Tprev[1] < 0.0)
				Tprev[1] = 0.0;*/
		
			/*M3_DEBUG("Init motor model:\n");
			//M3_DEBUG("eA1_i00: %f\n", eA1_i(0,0));
			//M3_DEBUG("eA1_i11: %f\n", eA1_i(1,1));
			M3_DEBUG("eA1_i00: %f\n", eA1_i(0,0));
			M3_DEBUG("eA1_i11: %f\n", eA1_i(1,1));
			M3_DEBUG("diff: %f\n", diff);
			M3_DEBUG("winding: %f\n", Tprev[0]);
			M3_DEBUG("case: %f\n", Tprev[1]);*/
	      }
	    }
	    
	}
	
	// C1 = tau1/r1, C2 = tau2/r2, tau3 = r1*C2
	// A = [-1/tau1 1/tau1;1/tau3,-1/tau3-1/tau2]
	// B = [1/C1;0]
        //RT_TASK_FREQUENCY
	// eA1 = eye(2)
	// ser = eye(2)
	// for i = 1:5
	//	ser *= A*dt/i;
	//	eA1 += ser;
	// eA2 == inv(A)*(eA1-eye(2))*B
}

void M3MotorModel::ThermalShutdown(string config_filename, mReal ambient_temp)
{
	if (use_old_temp_values)
	{
		  time_t rawtime;
		  struct tm *timeinfo;
		  char buf[80];
		  
		  
		  time(&rawtime);
		  timeinfo = localtime(&rawtime);
		  strftime(buf,80,"%c",timeinfo);
		  string s = string(buf);
		  
		  // Write Old Temps to file
		  string temp_filename = config_filename.substr(0, config_filename.length()-4);
		  temp_filename += "_temps.yml";
				  
		  
		  mReal wind_temp = GetWindingTemp() - ambient_temp;
		  if (wind_temp < 0.0)
			wind_temp = 0.1;

		  mReal case_temp = GetCaseTemp() - ambient_temp;
		  if (case_temp < 0.0)
			case_temp = 0.1;

		  YAML::Emitter out;
		  out << YAML::BeginMap;
		  out << YAML::Key << "previous_temp_timestamp";
		  out << YAML::Value << s;
		  out << YAML::Key << "previous_winding_temp";
		  out << YAML::Value << wind_temp;
		  out << YAML::Key << "previous_case_temp";
		  out << YAML::Value << case_temp;
		  out << YAML::EndMap;
		  
		  WriteYamlDoc(temp_filename.c_str(), out);
		  
	}
		  //M3_DEBUG("writing: %s\n", temp_filename.c_str() );
		  /*
		  YAML::Node config = YAML::LoadFile(config_filename);		  
		  config["previous_temp_timestamp"] = s;
		  config["previous_winding_temp"] = GetWindingTemp();
		  config["previous_case_temp"] = GetCaseTemp();

		  std::ofstream fout(config_filename);
		  fout << config;
		  */
}

void M3MotorModel::ReadConfig(const YAML::Node & doc, string config_filename)
{
	string t;
	doc["name"] >> name;
	
		
	try 
	{
		doc["model_type"] >> t;
		if (t.compare("none")==0){model_type=NONE; return;}
		if (t.compare("model_v0")==0){model_type=MODEL_V0;}
		if (t.compare("model_v1")==0){model_type=MODEL_V1;}
		if (t.compare("model_v2")==0){model_type=MODEL_V2;}
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		//M3_WARN("Missing version key in config file for motor %s. Defaulting to type MODEL_V0\n",name.c_str());
		model_type=MODEL_V0;
	} 
	
	if (model_type==MODEL_V0)
	{
		doc["winding_resistance"] >>winding_resistance;
		doc["thermal_resistance_housing_ambient"] >>thermal_resistance_housing_ambient;
		doc["thermal_resistance_rotor_housing"] >>thermal_resistance_rotor_housing;
		doc["max_winding_temp"] >>max_winding_temp;
		doc["gear_ratio"] >>gear_ratio;
		doc["thermal_time_constant_winding"] >>thermal_time_constant_winding; 
		current_sensor_type=CURRENT_MEASURED; //standard for RBL and previous versions
		temp_sensor_type=TEMP_CASE; //standard for RBL and previous versions
		//The winding temp will take thermal_time_constant_winding seconds to achieve a steady-state calculated value
		winding_temp_avg.Resize(1.0/(mReal)RT_TASK_FREQUENCY, thermal_time_constant_winding);
	}
	if (model_type==MODEL_V1 || model_type==MODEL_V2)
	{
		doc["nominal_voltage"] >> nominal_voltage;
		doc["no_load_speed"] >> no_load_speed;
		doc["no_load_current"] >> no_load_current;
		doc["nominal_speed"] >> nominal_speed;
		doc["nominal_torque"] >> nominal_torque;
		doc["nominal_current"] >> nominal_current;
		doc["stall_torque"] >> stall_torque;
		doc["starting_current"] >> starting_current;
		doc["max_efficiency"] >> max_efficiency;
		doc["winding_resistance"] >> winding_resistance;
		doc["winding_inductance"] >> winding_inductance;
		doc["torque_constant"] >> torque_constant;
		doc["speed_constant"] >> speed_constant;
		doc["speed_torque_gradient"] >> speed_torque_gradient;
		doc["mechanical_time_constant"] >> mechanical_time_constant;
		doc["rotor_inertia"] >> rotor_inertia;
		doc["thermal_resistance_housing_ambient"] >> thermal_resistance_housing_ambient;
		doc["thermal_resistance_rotor_housing"] >> thermal_resistance_rotor_housing;
		doc["thermal_time_constant_winding"] >> thermal_time_constant_winding;
		doc["thermal_time_constant_motor"] >> thermal_time_constant_motor;
		doc["max_winding_temp"] >> max_winding_temp;
		doc["gear_ratio"] >> gear_ratio;
		doc["max_pwm_duty"] >> max_pwm_duty;
		safe_pwm_duty=max_pwm_duty;
		safe_pwm_slew.Reset(max_pwm_duty);
		try 
		{
			doc["alpha_cu"]>>alpha_cu;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			alpha_cu=.00393; //copper constant
		} 
		try 
		{
			doc["safe_thermal_pct"]>>safe_thermal_pct;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			safe_thermal_pct=0.80;
		} 
		try 
		{
			doc["safe_pwm_pct"]>>safe_pwm_pct;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			safe_pwm_pct=0.90;
		} 
		try 
		{
			doc["i_scale"]>>i_scale;
		} catch(YAML::TypedKeyNotFound<string> e) 
		{
			i_scale=1.0;
		} 
		doc["amplifier_resistance"] >> amplifier_resistance;
		doc["current_sensor_type"]>>t;
		if (t.compare("none")==0){current_sensor_type=CURRENT_NONE;} 
		if (t.compare("measured")==0){current_sensor_type=CURRENT_MEASURED;} 
		if (t.compare("controlled")==0){current_sensor_type=CURRENT_CONTROLLED;} 
		doc["temp_sensor_type"]>>t;
		if (t.compare("none")==0){temp_sensor_type=TEMP_NONE;} 
		if (t.compare("ambient")==0){temp_sensor_type=TEMP_AMBIENT;} 
		if (t.compare("case")==0){temp_sensor_type=TEMP_CASE;} 
		//The winding temp will take thermal_time_constant_winding seconds to achieve a steady-state calculated value
		winding_temp_avg.Resize(1.0/(mReal)RT_TASK_FREQUENCY, thermal_time_constant_winding);
		case_temp_avg.Resize(1.0/(mReal)RT_TASK_FREQUENCY, thermal_time_constant_motor);
	}
	//1/2 sec for rms current estimates.
	//5 sec for continuous current estimates
	int window_us=0.5*1000000;
	int downsample = (window_us/RT_TASK_FREQUENCY)/100;
	curr_avg_rms.Resize(window_us, downsample);
	window_us=5.0*1000000;
	downsample = (window_us/RT_TASK_FREQUENCY)/100;
	curr_avg_cont.Resize(window_us, downsample);
	//Report back a 1s filtered temp sensor
	window_us=1.0*1000000;
	downsample = (window_us/RT_TASK_FREQUENCY)/100;
	tmp_avg.Resize(window_us,downsample);
	
	if (model_type==MODEL_V2)
	{
	  try 
	{
		doc["use_old_temp_values"] >> use_old_temp_values;	
	} catch(YAML::TypedKeyNotFound<string> e) 
	{
		//M3_WARN("Missing version key in config file for motor %s. Defaulting to type MODEL_V0\n",name.c_str());
		use_old_temp_values = false;
	} 
	  ThermalInit(config_filename);
	}
}


void M3MotorModel::Step(mReal i, mReal pwm, mReal rpm, mReal tmp)
{
	if (model_type==NONE)
	  return;
	if (model_type==MODEL_V0)
	  StepModelV0(i, pwm, rpm, tmp);
	if (model_type==MODEL_V1)
	  StepModelV1(i, pwm, rpm, tmp);
	if (model_type==MODEL_V2)
	  StepModelV2(i, pwm, rpm, tmp);
}

//Input sensor data of current sensor, amplifier pwm, velocity, and temp sensor
void M3MotorModel::StepModelV2(mReal i, mReal pwm, mReal rpm, mReal tmp)
{
	//Voltage
	mReal duty=0;
	v_pwm = 0;
	v_cemf =0;
	power_mech=0;
	power_elec=0;
	duty=pwm/max_pwm_duty ;
	v_pwm = ABS(nominal_voltage*pwm/max_pwm_duty);
	v_cemf = ABS(rpm*gear_ratio/speed_constant);
	
	if (current_sensor_type==CURRENT_NONE)
	{
		//Basic motor model with back emf. Estimates current from applied duty cycle and motor velocity.
		//See paper: "Towards a dynamic actuator model for a hexapod robot"	
		//I_a = (d*Vs-Ks*omega)/(Ra+d*d*Ramp)
		//duty should be -1.0 to 1.0
		//put in mA
		
		mReal I_ma = MAX(.00001,((MAX(0.0,v_pwm-v_cemf))/(winding_resistance+duty*duty*amplifier_resistance))*1000.0);
		mReal x=curr_avg_rms.Step(I_ma*I_ma); 
		i_rms = i_scale*sqrt(ABS(x)); //min(starting_current*1000.0,sqrt(curr_avg_rms.Step(I_ma*I_ma)));
		/*if (tmp_cnt++%100==0)
			M3_INFO("i_rms %f x %f I_ma %f v_pwm %f v_cemf %f duty %f\n",i_rms,x,I_ma, v_pwm, v_cemf, duty);
		i_cont=curr_avg_cont.Step(i_rms);*/
		i = I_ma;
	}
	
	//Current
	//i_rms=min(starting_current*1000.0,sqrt(ABS(curr_avg_rms.Step(i*i)))); //Just filter (Avoid NAN)
	i_rms=min(starting_current*1000.0,i); // Lee: note this model does not use rms
	i_cont=curr_avg_cont.Step(i_rms);

	//Winding resistance
	mReal hot_resistance = winding_resistance*(1.0+alpha_cu*(winding_temp-25.0));
	
	//Power
	power_mech=(i_rms/1000.0)*v_cemf;
	power_elec=(i_rms/1000.0)*v_pwm;
	power_heat=(i_rms/1000.0)*(i_rms/1000.0)*hot_resistance;

	//Temp
	if (first_wt_step)
		tmp_avg.Reset(25.0);
	ambient_temp=tmp_avg.Step(tmp);
	
	Tprev = eA1*Tprev+eA2*power_heat;
	winding_temp=ambient_temp+Tprev(0);
	case_temp=ambient_temp+Tprev(1);

	first_wt_step=MAX(0,first_wt_step-1);
	StepVoltageLimit();
}


//////////////////////////////////////////////////////////////////////
//Using cold resistance
mReal  M3MotorModel::mNmToPwm(mReal mNm)
{
    mReal i_a = (mNm/torque_constant)/gear_ratio;


    mReal duty = (i_a*winding_resistance+v_cemf)/nominal_voltage;
    duty=CLAMP(duty,-1.0,1.0);
    mReal pwm = duty*max_pwm_duty;
    //if (tmp_cnt++%100==0)
    //  M3_INFO("mNm: %f iA: %f v_cemf: %f vreq: %f, duty: %f pwm %f\n", mNm, i_a, v_cemf, i_a*winding_resistance, duty, pwm);
    return pwm;
}

void M3MotorModel::StepVoltageLimit()
{
	//When the winding is thermal_limit_pct% of over-temp, limit the voltage to the motor
	//to a value that results in the safe y% of max continuous current
	//slew to the limit over thermal_time_constant_winding seconds 
	//restore to max over 5 x thermal_time_constant_winding seconds (allow extra time for motor to cool down)
	mReal mpa=safe_pwm_pct*max_pwm_duty*nominal_torque/stall_torque;
	mReal rate_down = (max_pwm_duty-mpa)/MIN(thermal_time_constant_winding,1.0);
	mReal rate_up = (max_pwm_duty-mpa)/(5*thermal_time_constant_winding);
	if (winding_temp>safe_thermal_pct*max_winding_temp)
	{
		safe_pwm_duty=safe_pwm_slew.Step(mpa,rate_down);
	}
	else
		safe_pwm_duty=safe_pwm_slew.Step(max_pwm_duty,rate_up);
}

////////////////////////// LEGACY /////////////////////////////////////
//Keep around older (less accurate) models so thermal safety behavior of 
//older robots doesn't change


//Model V1 generally has an ambient temp sensor near the motor
//and may or may not have a useful current sensor value
//If it does not have a current sensor, the current is estimated from the 
//applied voltage and back-emf. The winding temp is approximated with
//an exponential filter, but the time constants are approximate
//Voltage limiting was added to this generation for additional safety.
void M3MotorModel::StepModelV1(mReal i, mReal pwm, mReal rpm, mReal tmp)
{
	/////////////// Voltage terms //////////////////////////////
	mReal duty=0;
	v_pwm = 0;
	v_cemf =0;
	power_mech=0;
	power_elec=0;
	duty=pwm/max_pwm_duty ;
	v_pwm = ABS(nominal_voltage*pwm/max_pwm_duty);
	v_cemf = ABS(rpm*gear_ratio/speed_constant);
	/////////////// Estimate current ////////////////////////////
	//Current controlled amplifier, Use the commanded value as the 'sensed' value
	if (current_sensor_type==CURRENT_CONTROLLED)
	{
		i_rms=i;
		i_cont=i;
	}
	//Current measurement available from the amplifier
	//i_cont is longer time scale than i_rms
	if (current_sensor_type==CURRENT_MEASURED)
	{
	  i_rms=min(starting_current*1000.0,sqrt(ABS(curr_avg_rms.Step(i*i)))); //Just filter (Avoid NAN)
	  i_cont=curr_avg_cont.Step(i_rms);
	}
	//No current sensing available. Estimate it based on the applied voltage and back-EMF
	if (current_sensor_type==CURRENT_NONE)
	{
		//Basic motor model with back emf. Estimates current from applied duty cycle and motor velocity.
		//See paper: "Towards a dynamic actuator model for a hexapod robot"	
		//I_a = (d*Vs-Ks*omega)/(Ra+d*d*Ramp)
		//duty should be -1.0 to 1.0
		//put in mA
		
		mReal I_ma = MAX(.00001,((MAX(0.0,v_pwm-v_cemf))/(winding_resistance+duty*duty*amplifier_resistance))*1000.0);
		mReal x=curr_avg_rms.Step(I_ma*I_ma); 
		i_rms = i_scale*sqrt(ABS(x)); //min(starting_current*1000.0,sqrt(curr_avg_rms.Step(I_ma*I_ma)));
		/*if (tmp_cnt++%100==0)
			M3_INFO("i_rms %f x %f I_ma %f v_pwm %f v_cemf %f duty %f\n",i_rms,x,I_ma, v_pwm, v_cemf, duty);
		i_cont=curr_avg_cont.Step(i_rms);*/
	}
	power_mech=(i_rms/1000.0)*v_cemf;
	power_elec=(i_rms/1000.0)*v_pwm;
	
	/////////////// Estimate winding temp ///////////////////////
	//Get Case/Ambient temp
	ambient_temp=25.0; //provide default in case no sensor present
	case_temp=25.0;
	winding_temp=25.0;
	
	if (first_wt_step)
		tmp_avg.Reset(25.0);
		
	//Newer hardware has ambient temp sensors
	if (temp_sensor_type==TEMP_AMBIENT) 
	{
		if (first_wt_step)
		{
			tmp_avg.Reset(tmp);
		}
		ambient_temp=tmp_avg.Step(tmp);
	}
	
	//Older hardware has case temp sensors
	if (temp_sensor_type==TEMP_CASE)
	{
		if (first_wt_step)
			tmp_avg.Reset(tmp);
		case_temp=tmp_avg.Step(tmp);
		ambient_temp=case_temp; //w/o greater knowledge, assume that internal body temp follows motor case temp
		
	}
	
	//Estimate winding resistance (from maxon catalog)
	mReal hot_resistance = winding_resistance*(1.0+alpha_cu*(winding_temp-25.0));
	//Estimate winding temp
	power_heat=(i_rms/1000.0)*(i_rms/1000.0)*hot_resistance;
	if (temp_sensor_type==TEMP_CASE)
	{
		mReal val=(thermal_resistance_rotor_housing)*power_heat+case_temp;
		if (first_wt_step)
			winding_temp_avg.Reset(val);
		winding_temp = winding_temp_avg.Step(val);
	}
	
	if ((temp_sensor_type==TEMP_AMBIENT || temp_sensor_type==TEMP_NONE))
	{
		mReal val_c=ambient_temp+thermal_resistance_rotor_housing*power_heat;
		mReal val_w=(thermal_resistance_rotor_housing+thermal_resistance_housing_ambient)*power_heat+ambient_temp;
		if (first_wt_step)
		{
			winding_temp_avg.Reset(val_w);
			case_temp_avg.Reset(val_c);
		}
		case_temp = case_temp_avg.Step(val_c); //Exponential filter with thermal time constant
		winding_temp = winding_temp_avg.Step(val_w);//Exponential filter with thermal time constant
	}
	first_wt_step=MAX(0,first_wt_step-1);//Resets for first few cycles to ensure start with valid data
	StepVoltageLimit();
}

//Model V0 generally has a temp sensor on the case of the motor
//Currents are measured from sensors, although this gen. or robot
//current sensing may be innacurate.
//Voltage limiting was added to this generation for additional safety.
void M3MotorModel::StepModelV0(mReal i, mReal pwm, mReal rpm, mReal tmp)
{
	/////////////// Voltage terms //////////////////////////////
	power_mech=0;
	power_elec=0;
	/////////////// Estimate current ////////////////////////////
	//Current controlled amplifier with no feedback, Use the commanded value as the 'sensed' value
	if (current_sensor_type==CURRENT_CONTROLLED)
	{
		i_rms=i;
		i_cont=i;
	}
	//Current measurement available from the amplifier
	//i_cont is longer time scale than i_rms
	if (current_sensor_type==CURRENT_MEASURED)
	{
	  i_rms=sqrt(ABS(curr_avg_rms.Step(i*i))); //Just filter (Avoid NAN)
	  i_cont=curr_avg_cont.Step(i_rms);
	}
	
	/////////////// Estimate winding temp ///////////////////////
	//Get Case/Ambient temp
	ambient_temp=25.0; //provide default in case no sensor present
	case_temp=25.0;
	winding_temp=25.0;

	if (first_wt_step)
		tmp_avg.Reset(25.0);
		
	//Newer hardware has ambient temp sensors
	if (temp_sensor_type==TEMP_AMBIENT) 
	{
		if (first_wt_step)
		{
			tmp_avg.Reset(tmp);
		}
		ambient_temp=tmp_avg.Step(tmp);
	}
	
	//Older hardware has case temp sensors
	if (temp_sensor_type==TEMP_CASE)
	{
		if (first_wt_step)
			tmp_avg.Reset(tmp);
		case_temp=tmp_avg.Step(tmp);
		ambient_temp=case_temp; //w/o greater knowledge, assume that internal body temp follows motor case temp
		
	}
	//Estimate winding resistance (from maxon catalog)
	mReal hot_resistance = winding_resistance*(1.0+alpha_cu*(winding_temp-25.0));
	//Estimate winding temp
	power_heat=(i_rms/1000.0)*(i_rms/1000.0)*hot_resistance;
	if (temp_sensor_type==TEMP_CASE)
	{
		mReal val=(thermal_resistance_rotor_housing)*power_heat+case_temp;
		if (first_wt_step)
			winding_temp_avg.Reset(val);
		winding_temp = winding_temp_avg.Step(val);
	}
	first_wt_step=MAX(0,first_wt_step-1);//Resets for first few cycles to ensure start with valid data
	StepVoltageLimit();
}



} //namespace
///////////////////////////////////////////////////////
