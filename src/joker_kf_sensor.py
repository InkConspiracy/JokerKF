#!/usr/bin/env python
import numpy as np
import scipy as sp
import sympy as sy

from sympy import symbols, diff 

symbol_names = [
    'global_x', #x position map frame
    'global_y', #y position map frame
    'global_theta', #angular position map frame
    'odom_x', #robot sensor x
    'odom_y', #robot sensor y
    'odom_theta', #robot sensor theta
    'odom_velocity_x', #x velocity in robot frame
    'odom_velocity_y', #y velocity in robot frame
    'odom_omega_z', #rotational velocity in robot frame
    'odom_scale_omega_z', #Imu parameter estimation scale for rotation 
    'odom_scale_vel_x', #imu parameter estimation scale acceleration in x
    'odom_scale_vel_y', #imu parameter estimation scale acceleration in y
    
]


vars_ = symbols(' '.join(symbol_names))
gx, gy, gt, ox, oy, ot, ovx, ovy, owz, osw, osx, osy
symbols = {}
for index, name in enumerate(symbol_names):
    symbols[name] = vars_[index]

sym_model = sp.sqrt(((gx + (ox+ovx*osx)) - pose[0,0])**2 +((gy+(oy+ovy*osy))- pose[0,1])**2)
sym_pose_j = {}

for i in symbol_names:
    sym_pose_j[i] = diff(sym_model, symbols[i])

for i in symbol_names[:5]:
    func_model[i] = lambdify(vars_ ,sym_model[i], 'numpy')

func_pose_J = {}
for i in range(0,state_size):
    func_pose_J[i] = lambdify(vars_ ,sym_pose_J[i], 'numpy')

magic_args = [kwargs[symbol_name] for symbol_name in symbol_names]
jacobian = np.zeros((len(symbol_names[:-6]), len(symbol_names[:-6]),))
for  i in symbol_names[:-6]:
     for j in symbol_names[:-6]:
        jacobian[i,j] = func_pose_J[i][j](*magic_args)


