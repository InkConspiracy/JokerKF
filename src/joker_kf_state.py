#!/usr/bin/env python
import numpy as np
import scipy as sp
import sympy as sy

from sympy import symbols, diff,lambdify

symbol_names = [
    'x', #x position map frame
    'y', #y position map frame
    'th', #angular position map frame
    'vx', #x velocity in robot frame
    'vy', #y velocity in robot frame
    'wz', #rotational velocity in robot frame
    'swz', #Imu parameter estimation scale for rotation 
    'sax', #imu parameter estimation scale acceleration in x robobt frame
    'say', #imu parameter estimation scale acceleration in y robot frame
    'bwz', #Imu parameter estimation bias for rotation 
    'bax', #imu parameter estimation bias acceleration in x robot frame
    'bay', #imu parameter estimation bias acceleration in y robot frame
    'ith', #imu sensor reading position
    'iwz', #imu sensor reading rotation 
    'iax', #imu sensor reading acceleration in x robot frame
    'iay', #imu sensor reading acceleration in y robot frame
    'dt', #time increment
]
   


THETA_GAIN = 0.1 # proportional value bias against new IMU data
vars_ = symbols(' '.join(symbol_names))
x, y, th, vx, vy, wz, swz, sax, say, bwz, bax, bay, ith, iwz, iax, iay, dt = vars_
state_size = len(vars_)-5
control_param = 2
symbols = {}
for index, name in enumerate(symbol_names):
    symbols[name] = vars_[index]
sym_motion = {
    'x' : x + dt*vx*sy.cos(th)-dt*vy*sy.sin(th),
    'y' : y + dt*vx*sy.sin(th)-dt*vy*sy.cos(th),
    'th' : th + (th + dt*wz)*THETA_GAIN + ith*(1-THETA_GAIN),
    'vx' : vx + (sax*iax+bax)*dt,
    'vy' : vy + (say*iay+bay)*dt,
    'wz' : swz*iwz + bwz,
    'swz' : swz,
    'sax' : sax,
    'say' : say,
    'bwz' : bwz,
    'bax' : bax,
    'bay' : bay,
    'ith' : ith,
    'iwz' : iwz,
    'iax' : iax,
    'iay' : iay,
    'dt' : dt,


}

# sym_pose_J = []
# for i in symbol_names[:-1]:
#     sym_pose_J[i] = []
#     for j in symbol_names[:-1]:
#         sym_pose_J[i][j] = diff(sym_motion[i],symbols[j])

m = map_ = symbol_names

func_pose_J =  []
for i,name1 in enumerate(symbol_names[:-5]):
    func_pose_J.append([])
    for j,name2 in enumerate(symbol_names[:-5]):
        func_pose_J[i].append([])
        func_pose_J[i][j] = lambdify(vars_ ,diff(sym_motion[m[i]],symbols[m[j]]), 'numpy')

func_motion = []
for i,name_ in enumerate(symbol_names[:-5]):
    func_motion.append(None)
    func_motion[i] = lambdify(vars_ ,sym_motion[name_], 'numpy')

def model_pose_jacobian( everything):

    
    jacobian = np.zeros((len(symbol_names[:-5]), len(symbol_names[:-5]),))
    for  i,name1 in enumerate(symbol_names[:-5]):
        for j,name2 in enumerate(symbol_names[:-5]):
            jacobian[i,j] = func_pose_J[i][j](*everything)
    return jacobian

def build_kwargs(delta_ms, control, pose):
    kwargs = []
    for i in range(0, state_size):
        kwargs[symbol_names[i]] = pose[i,0]
        kwargs['dt'] = delta_ms
        return kwargs

def motion_model(everything):
    
    new_pose = np.zeros((len(symbol_names[:-5]), 1,))
    for  i,name1 in enumerate(symbol_names[:-5]):
        new_pose[i] = func_motion[i](*everything)
    return new_pose
def model_control_Jacobian( everything):
    '''
    derivative of motion model wrt control, considering current pose
    '''
    magic_list = [ 'ith', 'iwz','iax', 'iay']

    
    jacobian = np.zeros((len(symbol_names[:-5]), 4,))
    for  i,name1 in enumerate(symbol_names[:-5]):
        for j,name2 in enumerate(magic_list):
            jacobian[i,j] = func_pose_J[i][j](*everything)
    return np.matrix(jacobian)

def model_control_covariance(imu_reading):
    '''
    M_t is the noise in control space
    This is translated to actual estimated noise by the algorithm
    '''
    #TODO:NEEDS MAGIC NUMBER
    return np.matrix(np.eye(4))


def motion_update(last_state, imu_reading, delta_ms):
    pose = last_state['pose']
    cov = last_state['covariance']
    width = len(pose)+len(imu_reading)+1
    everything = np.zeros((width,1))
    print(pose.shape)
    everything[:len(pose),:] = pose
    everything[len(pose):-1,:] = imu_reading
    everything[-1,:] = delta_ms
    G_t1 = model_pose_jacobian(everything)
    V_t1 = model_control_Jacobian(everything)
    M_t1 = model_control_covariance(imu_reading)
    mu_bar = np.matrix(motion_model(everything))
    print('mu_bar')
    print(mu_bar.shape)
    si_bar = G_t1*cov*G_t1.T+V_t1*M_t1*V_t1.T
    new_state = {
    'pose': mu_bar,
    'covariance': si_bar,
    }
    return new_state

def vis_od_update(last_state,odometry_reading, delta_ms):
    return last_state
def individual_uwb_update(last_state,x,y,distance):


    return last_state


def uwb_update(last_state,uwb_reading, delta_ms):
    locations = [
    (0,0), 
    (7,0),
    (0,15),
    (7,15),
    ]

    for index,distance in uwb_reading:
        x,y = locations[index]
        last_state = individual_uwb_update(last_state,x,y,distance)

    return last_state



if __name__ == '__main__':
    init_state = {
    'pose':{
    'x':1, #x position map frame
    'y':2, #y position map frame
    'th':0, #angular position map frame
    'vx':1, #x velocity in robot frame
    'vy':0, #y velocity in robot frame
    'wz':0, #rotational velocity in robot frame
    'swz':1, #Imu parameter estimation scale for rotation 
    'sax':1, #imu parameter estimation scale acceleration in x robobt frame
    'say':1, #imu parameter estimation scale acceleration in y robot frame
    'bwz':0, #Imu parameter estimation bias for rotation 
    'bax':0, #imu parameter estimation bias acceleration in x robot frame
    'bay':0, #imu parameter estimation bias acceleration in y robot frame
    },
    #TODO FIND MAGIC NUMBERS
    'covariance': np.matrix(np.eye(12)),
}
    magic_args = [init_state['pose'][symbol_name] for symbol_name in symbol_names[:-5]]
    init_state['pose'] = np.matrix([magic_args]).reshape((12,1))
    imu_reading = {
    'ith':0, #imu sensor reading position
    'iwz':0, #imu sensor reading rotation 
    'iax':1, #imu sensor reading acceleration in x robot frame
    'iay':0, #imu sensor reading acceleration in y robot frame
    }
    imu_reading = [
    imu_reading['ith'],
    imu_reading['iwz'],
    imu_reading['iax'],
    imu_reading['iay'],
    ]
    imu_reading = np.matrix(imu_reading).reshape((4,1))
    print('hello asshats')
    new_state = motion_update(init_state,imu_reading,100) 
    print('hello dumbasses')
    new_state = motion_update(new_state,imu_reading,100) 

