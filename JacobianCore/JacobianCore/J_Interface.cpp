#include "J_Interface.h"
#include "JacobianCore.h"

#include <Windows.h>
#include <iostream>
#include "DebugCPP.h"

using namespace std;
using namespace torch;


//void Init()
//{
//	cout << "Interface Function 1" << endl;
//	
//	ostringstream oss;
//	oss << "Init" << std::endl;
//	Debug::Log(oss.str(), Color::Blue);
//
//
//}

void Interface_1()
{
	cout << "Interface 1 Function Execute" << endl;
	myTestClass.Init();
	myTestClass.compute_jacobian();
}