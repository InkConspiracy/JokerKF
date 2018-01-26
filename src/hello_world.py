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
    'scale_omega_z', #Imu parameter estimation scale for rotation 
    'scale_accel_x', #imu parameter estimation scale acceleration in x
    'scale_accel_y', #imu parameter estimation scale acceleration in y
    'bias_omega_z', #Imu parameter estimation bias for rotation 
    'bias_accel_x', #imu parameter estimation bias acceleration in x
    'bias_accel_y', #imu parameter estimation bias acceleration in y
    'imu_theta', #imu sensor reading position
    'imu_omega_z', #imu sensor reading rotation 
    'imu_accel_x', #imu sensor reading acceleration in x robot frame
    'imu_accel_y', #imu sensor reading acceleration in y robot frame
    'dt', #time increment
]


vars_ = symbols(' '.join(symbol_names))
x, y, th, vx, vy, wz, swz, sax, say, bwz, bax, bay, ith, iwz, iax, iay, dt = vars_
symbols = {}
for index, name in enumerate(symbol_names):
    symbols[name] = vars_[index]
print(type (symbols['omega_z']))

fun = y + sy.cos(th)

print(sy.simplify(diff(fun, th)))