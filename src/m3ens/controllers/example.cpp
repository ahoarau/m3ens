#include "m3ens/controllers/example.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"
using namespace std;
using namespace KDL;

namespace m3ens{
	
using namespace m3rt;
using namespace std;
using namespace m3;
		
///////////////////////////////////////////////////////


void M3Example::Startup()
{
	if (bot==NULL)
		SetStateError();
	else
		SetStateSafeOp();

}

void M3Example::Shutdown()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						  
bool M3Example::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	if (!M3Component::ReadConfig(filename))
		return false;
	doc["humanoid"] >> bot_name;
	double val;
	doc["param"]["max_fx"] >> val;
	param.set_max_fx(val);
	doc["param"]["max_fy"] >> val;
	param.set_max_fy(val);
	doc["param"]["max_fz"] >> val;
	param.set_max_fz(val);
	doc["command"]["fx"] >> val;
	command.set_fx(val);
	doc["command"]["fy"] >> val;
	command.set_fy(val);
	doc["command"]["fz"] >> val;
	command.set_fz(val);
	return true;
}

bool M3Example::LinkDependentComponents()
{
	//Need to find at least one arm
	bot=(M3Humanoid*) factory->GetComponent(bot_name);
	if (bot==NULL)
		M3_INFO("M3Humanoid component %s not found for component %s\n",bot_name.c_str(),GetName().c_str());
	if (bot==NULL)
		return false;
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3Example::StepStatus()
{
	status.set_foo(1.0);
}

void M3Example::StepCommand()
{
 	tmp_cnt++;
	if (!command.enable())
	{
	    
	    bot->SetMotorPowerOn();
	    Eigen::Matrix<double,6,1> wrench;
	    wrench[0]=CLAMP(command.fx(),-param.max_fx(),param.max_fx());
	    wrench[1]=CLAMP(command.fy(),-param.max_fy(),param.max_fy());
	    wrench[2]=CLAMP(command.fz(),-param.max_fz(),param.max_fz());
	    wrench[3]=0;
	    wrench[4]=0;
	    wrench[5]=0;

	    Eigen::MatrixXd  J = bot->GetJacobian(RIGHT_ARM);
	    Eigen::Matrix<double,7,1> torque;
	    for(int i=0;i<7;i++)
	      torque[i] = bot->GetTorque_mNm(RIGHT_ARM,i);
	    
	   // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> invJ = J.inverse();
	   // Eigen::Matrix<double,Eigen::Dynamic,1> F=invJ*torque;
	   // if (tmp_cnt%100==0)
	   //   M3_INFO("On: %f %f %f\n",F[0],F[1],F[2]);
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> JT=J.transpose();
	    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> tq=JT*wrench;
	    
	      for (int i=0;i<bot->GetNdof(RIGHT_ARM);i++)
	      {
// 		  ((M3HumanoidCommand*)bot->GetCommand())->mutable_right_arm()->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE_GC);
// 		  ((M3HumanoidCommand*)bot->GetCommand())->mutable_right_arm()->set_tq_desired(i, tq[i]);
// 		  ((M3HumanoidCommand*)bot->GetCommand())->mutable_right_arm()->set_q_stiffness(i, 1.0);
  		  bot->SetModeTorqueGc(RIGHT_ARM,i);
// 		bot->SetModeThetaGc(RIGHT_ARM,i);-->Fonctionne
// //   		  bot->SetTorque_mNm(RIGHT_ARM,i,tq[i]);
		  
// 		  bot->SetTorque_mNm(RIGHT_ARM,0,10000.0);
//  		  bot->SetThetaDeg(RIGHT_ARM,i,0.0);
  		  bot->SetStiffness(RIGHT_ARM,i,1.0);
  		  bot->SetSlewRateProportional(RIGHT_ARM,i,1.0);
		  if (tmp_cnt%100==0)
			M3_INFO("q%d T cmd: %f\n",i,tq[i]);
	      }
	}
	else
	{
	  //if (tmp_cnt%100==0)
	   //   M3_INFO("Off\n");
	  bot->SetMotorPowerOff();
	  for (int i=0;i<bot->GetNdof(RIGHT_ARM);i++)
		  bot->SetModeOff(RIGHT_ARM,i);
		
	}
}

}
