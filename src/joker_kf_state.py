#!/usr/bin/env python
import numpy as np
import scipy as sp
import sympy as sy

from sympy import symbols, diff 

symbol_names = [
    'x', #x position map frame
    'y', #y position map frame
    'theta', #angular position map frame
    'velocity_x', #x velocity in robot frame
    'velocity_y', #y velocity in robot frame
    'omega_z', #rotational velocity in robot frame
    'imu_scale_omega_z', #Imu parameter estimation scale for rotation 
    'imu_scale_accel_x', #imu parameter estimation scale acceleration in x robobt frame
    'imu_scale_accel_y', #imu parameter estimation scale acceleration in y robot frame
    'odom_scale_omega_z', #odom parameter estimation scale for rotation 
    'odom_scale_vel_x', #odom parameter estimation scale velocity in x robot frame
    'odom_scale_vel_y', #odom parameter estimation scale velocity in y robot frame
    'imu_bias_omega_z', #Imu parameter estimation bias for rotation 
    'imu_bias_accel_x', #imu parameter estimation bias acceleration in x robot frame
    'imu_bias_accel_y', #imu parameter estimation bias acceleration in y robot frame
    #'odom_bias_omega_z', #odom sensor reading rotation 
    #'odom_bias_accel_x', #odom sensor reading acceleration in x robot frame
    #'odom_bias_accel_y', #odom sensor reading acceleration in y robot frame
    'imu_theta', #imu sensor reading position
    'imu_omega_z', #imu sensor reading rotation 
    'imu_accel_x', #imu sensor reading acceleration in x robot frame
    'imu_accel_y', #imu sensor reading acceleration in y robot frame
    'dt', #time increment
]
    #IF YOU WANT TO INCLUDE AN ESTIMATION FOR BIAS IN ODOM READINGS, ADD TO THE LIST BELOW!
THETA_GAIN = 0.1 # proportional value bias against new IMU data
vars_ = symbols(' '.join(symbol_names))
x, y, th, vx, vy, wz, swz, sax, say, osw, osx, osy, bwz, bax, bay, ith, iwz, iax, iay, dt = vars_
symbols = {}
for index, name in enumerate(symbol_names):
    symbols[name] = vars_[index]
sym_motion = [
    'x' = x + dt*vx*sy.cos(th)-dt*vy*sy.sin(th),
    'y' = y + dt*vx*sy.sin(th)-dt*vy*sy.cos(th),
    'th' = th + (th + dt*wz)*THETA_GAIN + ith*(1-THETA_GAIN),
    'vx' = vx + (sax*iax+bax)*dt,
    'vy' = vy + (say*iay+bay)*dt,
    'wz' = swz*iwz + bwz,
    'swz' = swz,
    'sax' = sax,
    'say' = say,
    'osw' = osw,
    'osx' = osx,
    'osy' = osy,
    'bwz' = bwz,
    'bax' = bax,
    'bay' = bay,
    'ith' = ith,
    'iwz' = iwz,
    'iax' = iax,
    'iay' = iay,
    'dt' = dt,


]

# sym_pose_J = []
# for i in symbol_names[:-1]:
#     sym_pose_J[i] = []
#     for j in symbol_names[:-1]:
#         sym_pose_J[i][j] = diff(sym_motion[i],symbols[j])

m = map_ = symbol_names

func_pose_J =  []
for i in symbol_names[:-5]:
    func_pose_J[i] = []
    for j in symbol_names[:-5]:
        func_pose_J[i][j] = lambdify(vars_ ,diff(sym_motion[i],symbols[j]), 'numpy')


def pose_jacobian( **kwargs):

    magic_args = [kwargs[symbol_name] for symbol_name in symbol_names]
    jacobian = np.zeros((len(symbol_names[:-5]), len(symbol_names[:-5]),))
    for  i in symbol_names[:-5]:
        for j in symbol_names[:-5]:
            jacobian[i,j] = func_pose_J[i][j](*magic_args)
