# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 09:50:39 2022

@author: suhail
"""
from typing import Tuple, List
import numpy as np 
import pickle

from MR_env import MR_Env


def readfile(
    filename: str
    ) -> Tuple[
        np.ndarray, 
        np.ndarray, 
        np.ndarray, 
        np.ndarray, 
        np.ndarray,
        ]:
    """
    Reads a file and returns the data as a tuple of numpy arrays
    Input:
        filename: the name of the file to read
    Output:
        X: the x positions of the robot
        Y: the y positions of the robot
        alpha: the alpha angles of the robot
        time: the time of each data point
        freq: the frequency of the magnetic field
    """
    
    with open(filename, 'rb') as f:
        data = pickle.load(f)

    dataset = data[0]
    params = dataset['Track_Params(frame,error,current_pos,target_pos,alpha,time)']
    freq = data[2]
    X = []
    Y = []
    alpha = []
    time = []
    for i in range(0, len(params)):
        row = params[i]
        X.append(row[2][0])
        Y.append(row[2][1])
        alpha.append(row[4])
        time.append(row[5])
    print('Finished loading pickle file\n')
    X = np.array(X)
    Y = np.array(Y)
    alpha = np.array(alpha)
    time = np.array(time)

    return X,Y,alpha,time,freq

def run_sim(
    actions: List[Tuple[float, float]],
    init_pos: List[Tuple[float, float]] = None,
    noise_var: int = 1,
    a0: int = 1, 
    is_mismatched: bool = False
    ) -> Tuple[
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
    """
    Runs a simulation of the MR environment
    Input:
        actions: the actions to take in the simulation, alpha and frequency values
        init_pos: the initial position of the robot
        noise_var: noise variable for the simulation
        a0: the amplitude of the magnetic field
        is_mismatched: whether the magnetic field is mismatched
    Output:
        X: the x positions of the robot
        Y: the y positions of the robot
        alpha: the yaw angles of the robot
        time: the time of each data point
        freq: the frequency of the magnetic field
    """
    state_prime = np.empty((0, 2))
    states      = np.empty((0, 2))
    env         = MR_Env(number_of_agents = 1)
    state       = env.reset(
                        init = init_pos,
                        noise_var = noise_var,
                        a0 = a0, 
                        is_mismatched = is_mismatched
                        )
    # init
    # states      = np.append(states, env.last_pos, axis=0)
    # state_prime = np.append(state_prime, np.array([0,0]), axis=0)
    for action in actions:
        env.step(action[0], action[1])
        states      = np.append(states, np.array([env.last_positions[0]]), axis=0)
        state_prime = np.append(state_prime, np.array([env.state_primes[0]]), axis=0)
    X       = states[:,0]
    Y       = states[:,1]
    alpha   = actions[:,1]
    freq    = actions[:,0]
    time    = np.linspace(0, (len(X) - 1)/30.0, len(X)) # (np.arange(len(X))) / 30.0 #timestep is 1/30
    
    return X, Y, alpha, time, freq

def find_alpha_corrected(
    v_desired: float,
    gp: object,
    ) -> Tuple[
        Tuple[float, float],
        float,
        float,
        float,
        float,
    ]:
    """
    Finds tha alpha, mean, and variance of the GP at the desired velocity
    Input:
        v_desired: the desired velocity
        gp: the GP object
    Output:
        alpha and f_d: the alpha and frequency values
        muX: the mean of the GP on the x axis
        muY: the mean of the GP on the y axis
        sigX: the variance of the GP on the x axis
        sigY: the variance of the GP on the y axis
    """
    return gp.predict(v_desired)    


