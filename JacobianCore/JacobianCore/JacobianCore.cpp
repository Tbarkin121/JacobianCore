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

PlanarArm myTestClass;

PlanarArm::PlanarArm()
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

void PlanarArm::Init(uint16_t n_seg)
{
	cout << device << endl;

	num_segments = n_seg;
	joint_angles = torch::rand({ num_segments }, device);
	joint_angles.requires_grad_(true);
	joint_lengths = torch::ones({ num_segments }, device) / num_segments;
	x_pos = torch::zeros({ num_segments + 1 }, device);
	y_pos = torch::zeros({ num_segments + 1 }, device);
	x_target = torch::tensor({ 1.0 }, device);
	y_target = torch::tensor({ 1.0 }, device);
	dx = torch::tensor({ 0 }, device);
	dy = torch::tensor({ 0 }, device);
	weights = torch::ones({ num_segments, 1 }, device);
	weights[0] = 0.0;
	// Check
	cout << "Tensor: \n" << joint_angles << endl;
	cout << "Device: " << joint_angles.device() << endl;
	cout << "Requires grad: " << joint_angles.requires_grad() << endl;
}

void PlanarArm::forward_kinematics(void)
{
	//cout << "Entered Forward Kinematics" << endl;
	// Need to recreate x_pos and y_pos so we don't use pieces of the old graph
	x_pos = torch::zeros({ num_segments + 1 }, device);
	y_pos = torch::zeros({ num_segments + 1 }, device);

	//torch::Tensor Mat1 = torch::rand({ 2, num_segments }, device);
	//torch::Tensor pos = torch::matmul(Mat1, joint_angles);
	//x_pos.index({ -1 }) = pos[0];
	//y_pos.index({ -1 }) = pos[1];
	for (int s = 1; s <= num_segments; s++) {
		x_pos[s] = x_pos[s - 1] + joint_lengths[s - 1] * torch::cos(torch::sum(joint_angles.slice(0, 0, s)));
		y_pos[s] = y_pos[s - 1] + joint_lengths[s - 1] * torch::sin(torch::sum(joint_angles.slice(0, 0, s)));
	}
}

void PlanarArm::update_angle(torch::Tensor dtheta)
{
	{
		torch::NoGradGuard no_grad;
		joint_angles -= dtheta.view({ -1 });
	}
}
void PlanarArm::get_residual(void)
{
	//cout << "Entered Get Residual" << endl;
	dx = x_pos.index({ -1 }) - x_target;
	dy = y_pos.index({ -1 }) - y_target;
}

void PlanarArm::compute_jacobian(void)
{
	//cout << "Entered Compute Jacobian" << endl;
	// Compute forward kinematics
	forward_kinematics();
	get_residual();

	// Zero out the gradients
	if (joint_angles.grad().defined())
	{
		joint_angles.mutable_grad() = torch::empty({ num_segments }, device);
	}


	//dx.backward(torch::Tensor(), true);
	dx.backward();

	torch::Tensor jacobian_x = joint_angles.grad();

	// Zero out the gradients again for the next backward pass
	if (joint_angles.grad().defined())
	{
		joint_angles.mutable_grad() = torch::empty({ num_segments }, device);
	}

	try {
		dy.backward();
	}
	//catch (const c10::Error& e) {
	//    std::cerr << "Error during forward pass: " << e.what() << std::endl;
	//}
	catch (const std::exception& e) {
		std::cerr << "Standard exception: " << e.what() << std::endl;
	}
	
	torch::Tensor jacobian_y = joint_angles.grad();

	J = torch::cat({ jacobian_x.view({1, -1}), jacobian_y.view({1, -1}) }, 0);
	// Check
	//cout << "Tensor: " << J << endl;
	//cout << "Device: " << J.device() << endl;
	//cout << "Requires grad: " << J.requires_grad() << endl;
}

void PlanarArm::control_update(void)
{
	int m = 2;
	int n = num_segments;
	double gamma = 1.0;

	PlanarArm::compute_jacobian();



	auto JJT = torch::matmul(J, J.permute({ 1, 0 }));
	auto Im = torch::eye(m, torch::TensorOptions().dtype(J.dtype()).device(J.device()));
	auto R = torch::stack({ dx, dy }).view({ -1,1 });

	torch::Tensor M1 = torch::linalg::solve(JJT, J, true);  // Using torch::linalg::solve's solution as the returned value
	torch::Tensor M2 = torch::linalg::solve(JJT + gamma * gamma * Im, R, true);
	auto In = torch::eye(n, torch::TensorOptions().dtype(J.dtype()).device(J.device()));
	auto Zp = In - torch::matmul(J.permute({ 1, 0 }), M1);
	auto DeltaThetaPrimary = torch::matmul(J.permute({ 1, 0 }), M2);
	auto DeltaThetaSecondary = torch::matmul(Zp, joint_angles.view({ -1,1 }) * weights);
	auto DeltaTheta = DeltaThetaPrimary + DeltaThetaSecondary;
	PlanarArm::update_angle(DeltaTheta);

}

void PlanarArm::set_target(float x_targ, float y_targ)
{

	x_target = torch::tensor({ x_targ }, device);
	y_target = torch::tensor({ y_targ }, device);
}

torch::Tensor PlanarArm::get_positions(void)
{

	torch::Tensor result = torch::stack({ x_pos, y_pos }, 0);
	return result;
}