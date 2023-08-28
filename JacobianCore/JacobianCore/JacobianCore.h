// JacobianCore.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
#include <torch/torch.h>
// TODO: Reference additional headers your program requires here.

//class DLLTestClass
//{
//
//private:
//	torch::Device device = torch::kCPU;
//
//	uint16_t m;
//	uint16_t n;
//
//	torch::Tensor t1;
//	torch::Tensor t2;
//	float f1;
//	float f2;
//
//
//public:
//	void Init(void);
//
//};
//extern DLLTestClass myTestClass;


class PlanarArm
{
private:
	torch::Device device = torch::kCPU;

	uint16_t num_segments;

	torch::Tensor joint_angles;
	torch::Tensor joint_lengths;
	torch::Tensor x_pos;
	torch::Tensor y_pos;
	torch::Tensor x_targ;
	torch::Tensor y_targ;
	torch::Tensor dx;
	torch::Tensor dy;
	torch::Tensor weights;
	torch::Tensor J;


public:
	// Constructor
	PlanarArm(uint16_t n_seg);

	// Class Functions
	void Init(void);
	void forward_kinematics(void);
	void update_angle(torch::Tensor dtheta);
	void get_residual(void);
	void compute_jacobian(void);
	void control_update(void);


};


extern PlanarArm myTestClass;