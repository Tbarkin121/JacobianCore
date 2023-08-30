#include "J_Interface.h"
#include "JacobianCore.h"

#include <torch/torch.h>
#include <torch/script.h>

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
	myTestClass.Init(3);
	myTestClass.compute_jacobian();
}


void I_Init(uint16_t n_seg)
{
	myTestClass.Init(n_seg);
}

void I_SetTarget(float x_targ, float y_targ)
{
	myTestClass.set_target(x_targ, y_targ);
}

void I_Control_Update(void)
{
	myTestClass.control_update();
}

void I_Get_Pos(float x_pos[], float y_pos[])
{
	torch::Tensor pos_tensor = myTestClass.get_positions();

	Tensor x_tensor_cpu = pos_tensor[0].cpu().contiguous();
	Tensor y_tensor_cpu = pos_tensor[1].cpu().contiguous();

	vector<float> v_x(x_tensor_cpu.data_ptr<float>(), x_tensor_cpu.data_ptr<float>() + x_tensor_cpu.numel());
	vector<float> v_y(y_tensor_cpu.data_ptr<float>(), y_tensor_cpu.data_ptr<float>() + y_tensor_cpu.numel());

	std::copy(v_x.begin(), v_x.end(), x_pos);
	std::copy(v_y.begin(), v_y.end(), y_pos);
}

void I_Set_Gamma(float gam)
{
	myTestClass.set_gamma(gam);
}