#include "m3ens/controller_example/example.h"
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
	cout<<"reading file :"<<filename<<endl;
	if (!M3Component::ReadConfig(filename))
		return false;
	doc["humanoid"] >> bot_name;
	cout<<"reading hum :"<<bot_name<<endl;
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
	//status.set_foo(1.0);
}

void M3Example::StepCommand()
{
	tmp_cnt++;
	mReal time=tmp_cnt/250.0;
	mReal K = 35.0;
	mReal qdes_deg = -40.0 + K*sin(time);
	
	if (!(tmp_cnt%100))
		cout<<"Current sin : "<<qdes_deg<<endl;
	if (!command.enable())
	{
	bot->SetMotorPowerOn();
	int i=4;
	
	bot->SetModeThetaGc(LEFT_ARM,i);
	bot->SetStiffness(LEFT_ARM,i,1.0);
	bot->SetSlewRateProportional(LEFT_ARM,i,1.0);
	bot->SetThetaDeg(LEFT_ARM,i,qdes_deg);
	}
	else
	{
	bot->SetMotorPowerOff();
	for (int i=0;i<bot->GetNdof(LEFT_ARM);i++)
		bot->SetModeOff(LEFT_ARM,i);
		
	}
}

}
