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
        self.joint_angles = torch.rand(num_segments, requires_grad=True)
        self.joint_lengths =  torch.ones(num_segments, requires_grad=False)/num_segments
        self.xs = torch.zeros(num_segments+1, requires_grad=False)
        self.ys = torch.zeros(num_segments+1, requires_grad=False)
        self.x_targ=torch.tensor(-0.33, requires_grad=False)
        self.y_targ=torch.tensor(0.44, requires_grad=False)
        
        self.weights = torch.ones([num_segments,1])
        self.weights[0] = 0
                
        
        plt.close('all')
        xp = torch.cat((torch.tensor([0.0]), self.xs)).detach().cpu().numpy()
        yp = torch.cat((torch.tensor([0.0]), self.ys)).detach().cpu().numpy()
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.line1, = self.ax.plot(xp, yp, 'r-') # Returns a tuple of line objects, thus the comma
        self.line2, = self.ax.plot(self.x_targ,self.y_targ, 'o') # Returns a tuple of line objects, thus the comma
        
    def forward_kinematics(self):
        self.xs = torch.zeros(self.num_segments+1, requires_grad=False)
        self.ys = torch.zeros(self.num_segments+1, requires_grad=False)
        for s in range(1, self.num_segments+1):
            self.xs[s] = self.xs[s-1] + self.joint_lengths[s-1]*torch.cos(torch.sum(self.joint_angles[0:s]))
            self.ys[s] = self.ys[s-1] + self.joint_lengths[s-1]*torch.sin(torch.sum(self.joint_angles[0:s]))
                
        
    def get_residual(self):
        self.dx = self.xs[-1] - self.x_targ
        self.dy = self.ys[-1] - self.y_targ
        # error = torch.sqrt(dx**2 + dy**2)
        
    def compute_jacobian(self):
        # Compute forward kinematics
        self.forward_kinematics()
        self.get_residual()


        if self.joint_angles.grad is not None:
            self.joint_angles.grad = None

        self.dx.backward()
        self.jacobian_x = self.joint_angles.grad.clone()
        

        # Zero out the gradients before computing the next one        
        # self.joint_angles.grad = None
        if self.joint_angles.grad is not None:
            self.joint_angles.grad = None

        self.dy.backward()
        self.jacobian_y = self.joint_angles.grad.clone()
        
        self.J = torch.stack((env.jacobian_x, env.jacobian_y))

        
    def update_angles(self, dtheta):
        with torch.no_grad():
            self.joint_angles -= dtheta.view(-1)
        
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
     


        
    def control(self):
        m = 2
        n = self.num_segments
        gamma = 2
        
        self.compute_jacobian()
        
        JJT = torch.matmul(self.J, self.J.permute([1,0]))
        Im = torch.eye(m)
        R = torch.stack((env.dx, env.dy)).view(-1,1)
        M1 = torch.linalg.solve(JJT, self.J)
        M2 = torch.linalg.solve(JJT+gamma**2*Im, R)
        In = torch.eye(n)
        Zp = In - torch.matmul(self.J.permute([1,0]), M1)
        DeltaThetaPrimary = torch.matmul(self.J.permute([1,0]), M2)
        DeltaThetaSecondary = torch.matmul(Zp, self.joint_angles.view(-1,1) * self.weights)
        DeltaTheta = DeltaThetaPrimary + DeltaThetaSecondary        
        self.update_angles(DeltaTheta)
        
    def mouse_move(self, event):
        x, y = event.xdata, event.ydata
        if(x and y):
            self.x_targ = torch.tensor(x, dtype=torch.float, requires_grad=False)
            self.y_targ = torch.tensor(y, dtype=torch.float, requires_grad=False)
        
        
        
        
env = PlanarArm(5)

#%%

for i in range(100):
    ang = torch.tensor(i*torch.pi/180)
    # env.control(-2.0, -1.5)
    start = time.perf_counter()
    env.control()
    end = time.perf_counter()
    dt = end-start
    # print(f"Control Time : {dt}")
    print(f"end effector pos : ({env.xs[-1]},{env.ys[-1]})")
    env.plot()
    

