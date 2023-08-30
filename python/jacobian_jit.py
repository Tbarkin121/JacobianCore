# -*- coding: utf-8 -*-
"""
Created on Sun Aug 27 18:30:17 2023

@author: Plutonium
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Aug 25 01:37:05 2023

@author: Plutonium
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 09:58:15 2023

@author: Plutonium
"""


import torch
import torch.nn as nn
import torch.nn.functional as F

import matplotlib.pyplot as plt
import numpy as np
import time
import torchviz


class PlanarArm(nn.Module):
    def __init__(self, num_segments):
        print('Init Arm')
        super(PlanarArm, self).__init__()
    
        
        self.num_segments = torch.tensor(num_segments)
        self.joint_angles = torch.zeros(num_segments)
        self.joint_angles.requires_grad_()
        self.joint_lengths =  torch.ones(num_segments)
        self.xs = torch.zeros(num_segments)
        self.ys = torch.zeros(num_segments)
        self.x_targ=torch.tensor(1.0)
        self.y_targ=torch.tensor(1.0)
        self.dx=torch.tensor(0.0)
        self.dy=torch.tensor(0.0)
        self.J=torch.zeros([2,3])
        self.J_inv=torch.zeros([2,3])
        self.J_inv_damped=torch.zeros([2,3])
        self.delta_theta=torch.zeros([3])
        self.weights = torch.tensor([[0],[1],[1]])
        
        plt.close('all')
        xp = torch.cat((torch.tensor([0.0]), self.xs)).detach().cpu().numpy()
        yp = torch.cat((torch.tensor([0.0]), self.ys)).detach().cpu().numpy()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.ax.set_xlim([-5, 5])
        self.ax.set_ylim([-5, 5])
        self.line1, = self.ax.plot(xp, yp, 'r-') # Returns a tuple of line objects, thus the comma
        self.line2, = self.ax.plot(self.x_targ,self.y_targ, 'o') # Returns a tuple of line objects, thus the comma
    
    @torch.jit.export
    def forward_kinematics(self):
        self.xs = torch.zeros(self.num_segments)
        self.ys = torch.zeros(self.num_segments)
        for s in range(int(self.num_segments.item())):
            if (s==0):
                self.xs[s] = self.joint_lengths[s]*torch.cos(self.joint_angles[s])
                self.ys[s] = self.joint_lengths[s]*torch.sin(self.joint_angles[s])
            else:
                self.xs[s] = self.xs[s-1] + self.joint_lengths[s]*torch.cos(torch.sum(self.joint_angles[0:s+1]))
                self.ys[s] = self.ys[s-1] + self.joint_lengths[s]*torch.sin(torch.sum(self.joint_angles[0:s+1]))
                
        
    @torch.jit.export
    def compute_jacobian(self):
        # Compute forward kinematics
        self.forward_kinematics()
        self.get_residual()


        if (self.joint_angles.grad is not None):
            new_grad = torch.zeros(3)
            self.joint_angles.grad.set_(new_grad)
        # self.joint_angles.grad = None

            
        self.dx.backward()
        jacobian_x = [self.joint_angles.grad[0], self.joint_angles.grad[1], self.joint_angles.grad[2]]
        
        # Zero out the gradients before computing the next one
        if self.joint_angles.grad is not None:
            new_grad = torch.zeros(3)
            self.joint_angles.grad.set_(new_grad)
        # self.joint_angles.grad = None

        self.dy.backward()
        jacobian_y =[self.joint_angles.grad[0], self.joint_angles.grad[1], self.joint_angles.grad[2]]
        
        self.J = torch.tensor([jacobian_x, jacobian_y])
        # self.J = torch.stack(torch.autograd.functional.jacobian(fun, inputs),dim=0).squeeze()
        
    @torch.jit.export
    def update_angles(self, dtheta):
        with torch.no_grad():
            self.joint_angles -= dtheta
        
    @torch.jit.export
    def get_residual(self):
        self.dx = self.xs[-1] - self.x_targ
        self.dy = self.ys[-1] - self.y_targ
        # error = torch.sqrt(dx**2 + dy**2)

    @torch.jit.export
    def control_update(self):
        m = 2
        n = 3
        gamma = 5
        
        self.compute_jacobian()
        
        JJT = self.J * self.J.permute([1,0])
        Im = torch.eye(m)
        R = torch.stack((env.dx, env.dy)).view(-1,1)
        M1 = torch.solve(JJT, J)
        M2 = torch.solve(JJT+gamma**2*I, R)
        In = torch.eye(n)
        Zp = In - torch.matmul(J.permute([1,0]), M1)
        DeltaThetaPrimary = torch.matmul(J.permute([1,0]), M2)
        DeltaThetaSecondary = torch.matmul(Zp, self.joint_angles * self.weights)
        DeltaTheta = DeltaThetaPrimary + DeltaThetaSecondary        
        self.update_angles(DeltaTheta)
        
        
    @torch.jit.export
    def target_update(self, target):
        self.x_targ = target[0]
        self.y_targ = target[1]
        
    @torch.jit.export
    def get_angles(self):
        return self.joint_angles
    
    @torch.jit.export
    def get_positions(self):
        return torch.cat( (self.xs.view(-1,1), self.ys.view(-1,1)), dim=1)
    

    def plot(self):
        self.forward_kinematics()
        xp = torch.cat((torch.tensor([0.0]), self.xs)).detach().cpu().numpy()
        yp = torch.cat((torch.tensor([0.0]), self.ys)).detach().cpu().numpy()
        self.line1.set_xdata(xp)
        self.line1.set_ydata(yp)
        self.line2.set_xdata(self.x_targ)
        self.line2.set_ydata(self.y_targ)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.connect('motion_notify_event', self.mouse_move)        

    def mouse_move(self, event):
        x, y = event.xdata, event.ydata
        if(x and y):
            self.x_targ = torch.tensor(x, dtype=torch.float, requires_grad=False)
            self.y_targ = torch.tensor(y, dtype=torch.float, requires_grad=False)
        
env = PlanarArm(3)
#%%
env.forward_kinematics()
res = env.get_positions()
print(res)
#%%
# Convert the model to TorchScript via scripting
scripted_model = torch.jit.script(env)
#%%
# Save the scripted model
torch.jit.save(scripted_model, "PlanarArm.pt")
# scripted_model.save("PlanarArm.pt")
#%%


for i in range(1000):
    ang = torch.tensor(i*torch.pi/180)
    # env.control_update(-2.0, -1.5)
    env.control_update()
    env.plot()
    
