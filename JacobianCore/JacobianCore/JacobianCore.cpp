// JacobianCore.cpp : Defines the entry point for the application.
//

#include <Windows.h>
#include <torch/torch.h>
#include <torch/script.h>
#include <memory>
#include <iostream>
#include <chrono>

#include "JacobianCore.h"
#include "DebugCPP.h"

using namespace std;
using namespace std::chrono;

//DLLTestClass myTestClass;

//void DLLTestClass::Init()
//{
//	device = torch::kCPU;
//	if (torch::cuda::is_available()) {
//		std::cout << "CUDA is available! Training on GPU." << std::endl;
//		MessageBox(NULL, TEXT("Cuda Available"), TEXT("Third Party Plugin"), MB_OK);
//		device = torch::kCUDA;
//	}
//	cout << "My Device is :" << endl;
//	cout << device << endl;
//	cout << " !!!!!!!!!! " << endl;
//	
//	m = 1000;
//	n = 100;
//
//	t1 = torch::randn({ n, n }, device);
//	t2 = torch::randn({ n, m }, device);
//	f1 = 0.1;
//	f2 = 0.2;
//}


PlanarArm myTestClass(3);

PlanarArm::PlanarArm(uint16_t n_seg)
	: num_segments(n_seg)
{
	// Set device default to CPU
	device = torch::kCPU;
	 //Check for GPU availability
	if (torch::cuda::is_available()) {
		cout << "CUDA is available! Training on GPU." << endl;
		//MessageBox(NULL, TEXT("CUDA is available! Training on GPU."), TEXT("Third Party Plugin"), MB_OK);
		device = torch::kCUDA;
	}


}

void PlanarArm::Init(void)
{
	cout << device << endl;
	cout << "Test Start" << endl;
	torch::Tensor test = torch::zeros({ 3 }, device);
	cout << "Test End" << endl;
	joint_angles = torch::zeros({ num_segments }, device);
	joint_angles.requires_grad_(true);
	joint_lengths = torch::ones({ num_segments }, device);
	x_pos = torch::zeros({ num_segments }, device);
	y_pos = torch::zeros({ num_segments }, device);
	x_targ = torch::tensor({ 1 }, device);
	y_targ = torch::tensor({ 1 }, device);
	dx = torch::tensor({ 0 }, device);
	dy = torch::tensor({ 0 }, device);
	weights = torch::tensor({ {0}, {1}, {1} }, device);

	// Check
	cout << "Tensor: \n" << joint_angles << endl;
	cout << "Device: " << joint_angles.device() << endl;
	cout << "Requires grad: " << joint_angles.requires_grad() << endl;
}

void PlanarArm::forward_kinematics(void)
{
	cout << "Entered Forward Kinematics" << endl;
	// Need to recreate x_pos and y_pos so we don't use pieces of the old graph
	x_pos = torch::zeros({ num_segments }, device);
	y_pos = torch::zeros({ num_segments }, device);
	for (int s = 0; s < num_segments; s++) {
		if (s == 0) {
			x_pos[s] = joint_lengths[s] * torch::cos(joint_angles[s]);
			y_pos[s] = joint_lengths[s] * torch::sin(joint_angles[s]);
		}
		else {
			x_pos[s] = x_pos[s - 1] + joint_lengths[s] * torch::cos(torch::sum(joint_angles.slice(0, 0, s + 1)));
			y_pos[s] = y_pos[s - 1] + joint_lengths[s] * torch::sin(torch::sum(joint_angles.slice(0, 0, s + 1)));
		}
	}
	cout << x_pos << endl;
	cout << x_pos.index({ 0 }) << endl;
	cout << x_pos.index({ num_segments - 1 }) << endl;
	cout << x_pos.index({ -1 }) << endl;
}

void PlanarArm::update_angle(torch::Tensor dtheta)
{
	cout << "Entered Update Angle" << endl;
	joint_angles -= dtheta;
}
void PlanarArm::get_residual(void)
{
	cout << "Entered Get Residual" << endl;
	dx = x_pos.index({ -1 }) - x_targ;
	dy = y_pos.index({ -1 }) - y_targ;
}

void PlanarArm::compute_jacobian(void)
{
	cout << "Entered Compute Jacobian" << endl;
	// Compute forward kinematics
	forward_kinematics();
	get_residual();

	// Zero out the gradients
	cout << "F1" << endl;
	joint_angles.grad().zero_();
	cout << "F2" << endl;
	dx.backward();
	cout << "F3" << endl;
	vector<torch::Tensor> jacobian_x = { joint_angles.grad()[0], joint_angles.grad()[1], joint_angles.grad()[2] };
	cout << "F4" << endl;
	// Zero out the gradients again for the next backward pass
	joint_angles.grad().zero_();
	cout << "F5" << endl;
	dy.backward();
	cout << "F6" << endl;
	vector<torch::Tensor> jacobian_y = { joint_angles.grad()[0], joint_angles.grad()[1], joint_angles.grad()[2] };
	cout << "F7" << endl;
	J = torch::stack({ torch::stack(jacobian_x), torch::stack(jacobian_y) });
	cout << "F8" << endl;
	// Check
	cout << "Tensor: " << J << endl;
	cout << "Device: " << J.device() << endl;
	cout << "Requires grad: " << J.requires_grad() << endl;
}

//void PlanarArm::control_update(void)
//{
//	PlanarArm::compute_jacobian();
//	J_inv = torch::linalg::pinv(J);  // Assuming `env.J` was supposed to be `self.J`
//	double gamma = 5;
//	auto I = torch::eye(2, torch::dtype(torch::kFloat32).device(device));  // Assuming `device` is defined elsewhere in your code
//	J_inv_damped = torch::matmul(J.permute({ 1,0 }), torch::linalg::inv(torch::matmul(J, J.permute({ 1,0 })) + gamma * gamma * I));
//
//	int m = 2;
//	int n = 3;
//	auto N_mat = torch::eye(n, torch::dtype(torch::kFloat32).device(device)) - torch::matmul(J_inv, J);
//	auto pinvL_Nm_A = torch::linalg::lstsq(N_mat.slice(0, 0, m - n).slice(1, 0, m - n), N_mat.slice(0, 0, m - n))[0];
//	auto th_r = torch::matmul(torch::matmul(N_mat.slice(0, 0, m - n).permute({ 1,0 }), pinvL_Nm_A), joint_angles.view({ -1,1 }) * weights);
//	delta_theta = torch::matmul(J_inv_damped, torch::tensor({ {dx}, {dy} })) + th_r;
//
//}