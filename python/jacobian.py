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
import matplotlib.pyplot as plt
import numpy as np
import time
import torchviz


class PlanarArm:
    def __init__(self, num_segments):
        print('Init Arm')
        self.num_segments = num_segments
        self.joint_angles = torch.zeros(num_segments, requires_grad=True)
        self.joint_lengths =  torch.ones(num_segments, requires_grad=False)
        self.xs = torch.zeros(num_segments, requires_grad=False)
        self.ys = torch.zeros(num_segments, requires_grad=False)
        self.x_targ=torch.tensor(1.0, requires_grad=False)
        self.y_targ=torch.tensor(1.0, requires_grad=False)

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
        
    def forward_kinematics(self):
        self.xs = torch.zeros(self.num_segments, requires_grad=False)
        self.ys = torch.zeros(self.num_segments, requires_grad=False)
        for s in range(self.num_segments):
            if (s==0):
                self.xs[s] = self.joint_lengths[s]*torch.cos(self.joint_angles[s])
                self.ys[s] = self.joint_lengths[s]*torch.sin(self.joint_angles[s])
            else:
                self.xs[s] = self.xs[s-1] + self.joint_lengths[s]*torch.cos(torch.sum(self.joint_angles[0:s+1]))
                self.ys[s] = self.ys[s-1] + self.joint_lengths[s]*torch.sin(torch.sum(self.joint_angles[0:s+1]))
                
        
                 
    def compute_jacobian(self):
        # Compute forward kinematics
        self.forward_kinematics()
        self.get_residual()



        # self.joint_angles.grad = None
        if self.joint_angles.grad is not None:
            self.joint_angles.grad.set_(torch.zeros(3))
            # self.joint_angles.grad.zero_()

        print('!')
        self.dx.backward()
        print(self.joint_angles.grad)
        self.jacobian_x = [self.joint_angles.grad[0], self.joint_angles.grad[1], self.joint_angles.grad[2]]
        
        # Zero out the gradients before computing the next one

        self.forward_kinematics()
        self.get_residual()
        
        # self.joint_angles.grad = None
        if self.joint_angles.grad is not None:
            self.joint_angles.grad.set_(torch.zeros(3))
            # self.joint_angles.grad.zero_()

        self.dy.backward()
        print(self.joint_angles.grad)
        self.jacobian_y =[self.joint_angles.grad[0], self.joint_angles.grad[1], self.joint_angles.grad[2]]
        
        self.J = torch.tensor([self.jacobian_x, self.jacobian_y])
        # self.J = torch.stack(torch.autograd.functional.jacobian(fun, inputs),dim=0).squeeze()
        
    def update_angles(self, dtheta):
        with torch.no_grad():
            self.joint_angles -= dtheta
        
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
     
    def get_residual(self):
        self.dx = self.xs[-1] - self.x_targ
        self.dy = self.ys[-1] - self.y_targ
        # error = torch.sqrt(dx**2 + dy**2)

        
    def control(self):
        
        self.compute_jacobian()
        self.J_inv = torch.linalg.pinv(env.J)
        gamma = 5
        I = torch.eye(2)
        self.J_inv_damped = torch.matmul(self.J.permute([1,0]), torch.linalg.inv(torch.matmul(self.J, self.J.permute([1,0])) + gamma**2*I ))
        
        
        
        # DX_d2 = torch.tensor(self.joint_angles)
        # J_Jt = torch.matmul(self.J,self.J.permute([1,0]))
        # JpinvDamped = torch.matmul(J.permute([1,0]),torch.linalg.solve(J_Jt + gamma2*torch.eye(m),torch.eye(m)))
        
        m = 2
        n = 3
        N_mat = (torch.eye(n)- torch.matmul(self.J_inv, self.J))
        pinvL_Nm_A =  torch.linalg.lstsq(N_mat[:m-n,:m-n],N_mat[:m-n,:])[0]
        th_r = torch.matmul(torch.matmul(N_mat[:m-n,:].permute([1,0]),pinvL_Nm_A), self.joint_angles.view(-1,1)*self.weights)
        self.delta_theta = torch.matmul(self.J_inv_damped, torch.tensor([[self.dx], [self.dy]])) + th_r;
        
        # print(N_mat.shape)
        # print(pinvL_Nm_A.shape)
        # print(th_r.shape)
        # print(self.delta_theta.shape)
        

        # Calculate the desired change in joint angles
        # self.delta_theta = torch.matmul(self.J_inv, torch.tensor([self.dx, self.dy]))
        # self.delta_theta = torch.linalg.lstsq(self.J,torch.tensor([self.dx,self.dy]))[0] 

        self.update_angles(self.delta_theta.view(-1))
        
    def mouse_move(self, event):
        x, y = event.xdata, event.ydata
        if(x and y):
            self.x_targ = torch.tensor(x, dtype=torch.float, requires_grad=False)
            self.y_targ = torch.tensor(y, dtype=torch.float, requires_grad=False)
        
        
        
        
env = PlanarArm(3)

#%%

for i in range(1000):
    ang = torch.tensor(i*torch.pi/180)
    # env.control(-2.0, -1.5)
    env.control()
    env.plot()
    

