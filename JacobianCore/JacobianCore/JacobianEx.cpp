#include "JacobianCore.h"

using namespace std;
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

int main()
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	cout << "Hello CMake." << endl;


	myTestClass.Init(5);
	//myTestClass.compute_jacobian();
	myTestClass.set_target(-0.33, 0.44);
	torch::Tensor res;
	for (int i = 0; i < 100; i++)
	{
		start = std::chrono::system_clock::now();
		myTestClass.control_update();
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		cout << "Update Time : " << (float)elapsed_seconds.count() << endl;
		res = myTestClass.get_positions();
		cout << " End Effector Pos : " << endl << res.index({ torch::indexing::Slice(), -1 })<< endl << endl;
		//cout << res << endl;
	}
	

	return 0;
}

