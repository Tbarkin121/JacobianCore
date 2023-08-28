import torch
import torch.nn as nn
import torch.nn.functional as F  # Import functional module
import torch.jit as jit
# Define the model with control flow
class ControlFlowModel(nn.Module):
    def __init__(self):
        super(ControlFlowModel, self).__init__()
        self.fc1 = nn.Linear(10, 50)
        self.fc2 = nn.Linear(50, 1)
        self.threshold = 5.0

    def forward(self, x):
        if x.sum() > self.threshold:
            x = F.relu(self.fc1(x))
            
        else:
            x = F.elu(self.fc1(x))
            
        return self.fc2(x)

# Instantiate the model
model = ControlFlowModel()

# Convert the model to TorchScript via scripting
scripted_model = torch.jit.script(model)

# Save the scripted model
scripted_model.save("control_flow_model.pt")

#%%

import torch
import torch.nn.functional as F

@torch.jit.script
def standalone_relu_function(x):
    return F.relu(x)

# Save the scripted function
standalone_relu_function.save("relu_function.pt")


#%%

n = 10

class JacobianModule(nn.Module):
    def __init__(self):
        super(JacobianModule, self).__init__()


    def my_init(self, n):
        self.A = torch.rand([int(n[0].item()),int(n[0].item())])
        self.B = torch.rand([int(n[0].item()),1])
        
    def fun(self, x):
        R = torch.matmul(self.A,x) - self.B
        return R.squeeze()
    
    def forward(self, x, n):
        
        self.my_init(n)
        R = self.fun(x)
        back = R[0].backward()
        
        return (x.grad)


# Instantiate the model
model = JacobianModule()

test_input_x = torch.rand([3,1], requires_grad=True)
test_input_n = torch.tensor([3])
test_output = model(test_input_x, test_input_n)

print(test_input)
print(test_output)

#%%
# Convert the model to TorchScript via scripting
scripted_model = torch.jit.script(model)

#%%
# Save the scripted model
scripted_model.save("jacobian_model.pt")
#%%

# Convert the model to TorchScript via scripting
scripted_model = torch.jit.script(model)

# Save the scripted model
scripted_model.save("control_flow_model.pt")

@torch.jit.script 
def fun(x):
    R = A*X - B
    J = torch.autograd.functional.jacobian()    
    return R

