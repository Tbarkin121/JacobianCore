
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>

using namespace std;

int main()
{
    cout << "Hello CMake." << endl;
    //myTestClass.compute_jacobian();


     // Load the scripted modelz
    torch::jit::script::Module module;
    torch::jit::script::Module function;
    torch::jit::script::Module robot_arm;
    try {
        module = torch::jit::load("C:\\Users\\Plutonium\\MachineLearning\\git\\JacobianCore\\JacobianCore\\JacobianCore\\jit_models\\control_flow_model.pt");
        function = torch::jit::load("C:\\Users\\Plutonium\\MachineLearning\\git\\JacobianCore\\JacobianCore\\JacobianCore\\jit_models\\relu_function.pt");
        robot_arm = torch::jit::load("C:\\Users\\Plutonium\\MachineLearning\\git\\JacobianCore\\JacobianCore\\JacobianCore\\jit_models\\PlanarArm.pt");

        cerr << "Sooo.... it worked?\n";


    }
    catch (const c10::Error& e) {
        std::cerr << "Error loading the model\n";
        return -1;
    }

    // Create a sample input tensor and run it through the model
    at::Tensor input_tensor = torch::randn({ 1, 10 });
    at::Tensor function_input = torch::randn({ 10, 10 });


    try {
        at::Tensor output = module.forward({ input_tensor }).toTensor();

        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(function_input);
        at::Tensor result = function.get_method("forward")(inputs).toTensor();
        // Print the output
        //std::cout << "Input sum: " << inputs.sum().item<float>() << ", Model output: " << output.item<float>() << '\n';
        std::cout << "Model output: " << output.item<float>() << '\n';
        std::cout << result << std::endl;
    }
    //catch (const c10::Error& e) {
    //    std::cerr << "Error during forward pass: " << e.what() << std::endl;
    //}
    catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }

    cout << "F5" << endl;

    try {
        auto kinematics = robot_arm.find_method("forward_kinematics");
        auto get_angles = robot_arm.find_method("get_angles");
        auto get_positions = robot_arm.find_method("get_positions");
        auto update_angles = robot_arm.find_method("update_angles");
        auto control_update = robot_arm.find_method("control_update");
        auto target_update = robot_arm.find_method("target_update");

        cout << "Robot Arm Functions found :" << endl;
        for (const auto& method : robot_arm.get_methods()) {
            std::cout << method.name() << std::endl;
        }
        if (target_update) {
            cout << "Running Target Update" << endl;
            at::Tensor target = torch::tensor({ 1.0, 1.0 });
            (*target_update)({ target });
        }
        if (control_update) {
            cout << "Running Control Update" << endl;
            for (int i = 0; i < 100; i++)
            {
                (*control_update)({});
                if (get_positions) {
                    cout << "Running Get Position" << endl;
                    torch::IValue result = (*get_positions)({});
                    at::Tensor positions_tensor = result.toTensor();
                    cout << positions_tensor << endl;
                }
            }
        }

        //if (kinematics) {
        //    cout << "Running Kinematics" << endl;
        //    (*kinematics)({});
        //}
        //if (get_angles) {
        //    cout << "Running Get Angle" << endl;
        //    torch::IValue result = (*get_angles)({});
        //    at::Tensor angle_tensor = result.toTensor();
        //    cout << angle_tensor << endl;
        //}
        //if (get_positions) {
        //    cout << "Running Get Position" << endl;
        //    torch::IValue result = (*get_positions)({});
        //    at::Tensor positions_tensor = result.toTensor();
        //    cout << positions_tensor << endl;
        //}
        //if (update_angles) {
        //    cout << "Running Update Angles" << endl;
        //    at::Tensor dtheta = torch::rand({ 3 }); 
        //    (*update_angles)({ dtheta });
        //}
        //if (kinematics) {
        //    cout << "Running Kinematics" << endl;
        //    (*kinematics)({});
        //}
        //if (get_angles) {
        //    cout << "Running Get Angle" << endl;
        //    torch::IValue result = (*get_angles)({});
        //    at::Tensor angle_tensor = result.toTensor();
        //    cout << angle_tensor << endl;
        //}
        //if (get_positions) {
        //    cout << "Running Get Position" << endl;
        //    torch::IValue result = (*get_positions)({});
        //    at::Tensor positions_tensor = result.toTensor();
        //    cout << positions_tensor << endl;
        //}
        cout << "are we here?" << endl;
    }
    //catch (const c10::Error& e) {
    //    std::cerr << "Error during forward pass: " << e.what() << std::endl;
    //}
    catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }

    return 0;
}
