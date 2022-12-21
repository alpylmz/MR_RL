#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np
from scipy.integrate import RK45

from consts import NUMBER_OF_AGENTS
from typing import Tuple, List

class Simulator:
    def __init__(self, number_of_agents = 1):
        self.last_states = []
        self.current_action = None
        self.time_span = 0.030 #assume a timestep of 30 ms
        self.number_iterations = 100  #  iterations for each step
        self.integrators = [] # 
        ##MR Constants
        # TODO assume each robot is identical
        self.a0 = 0
        self.state_primes = []
        self.noise_var = 0
        self.is_mismatched = False
        self.number_of_agents = number_of_agents

    def reset_start_pos(
        self, 
        state_vectors: List[Tuple[float, float]]
        ) -> None:
        """
        Reset the start position of the simulator
        It changes the last state of the simulator, 
        resets the current action, and resets the integrator accordingly
        Input:
            state_vector: x and y position
        """  
        self.last_states = []
        self.current_action = []
        self.integrators = []
        self.current_action = np.zeros(2)
        for i in range(self.number_of_agents):
            x0, y0 = state_vectors[i][0], state_vectors[i][1]
            self.last_states.append(np.array([x0, y0]))
            self.integrators.append(
                self.scipy_runge_kutta(
                    lambda t, states: self.simulate(i, t, states), 
                    self.get_state(i), 
                    t_bound = self.time_span
                )
            )

    def step(
        self, 
        f_t: float, 
        alpha_t: float
        ) -> Tuple[
            float, 
            float,
        ]:
        """
        Step the simulator forward by one timestep
        Input:
            f_t: the frequency of the magnetic field
            alpha_t: the yaw angle of the robot
        Output:
            last_state: the new state of the robot, x and y position
        """
        self.current_action = np.array([f_t, alpha_t])
        # TODO: If any of the integrators is not finished, then it should be finished?
        # TODO: Or maybe we should put a different condition here?
        for i in range(self.number_of_agents):
            print(i)
            while not (self.integrators[i].status == 'finished'):
                self.integrators[i].step()
                

        for i in range(self.number_of_agents):
            self.last_states[i] = self.integrators[i].y
            self.integrators[i] = self.scipy_runge_kutta(
                                            lambda t, states: self.simulate(i, t, states),
                                            self.get_state(i),
                                            t0 = self.integrators[i].t,
                                            t_bound = self.integrators[i].t + self.time_span)

        return self.last_states


    def a0_linear(
        self, 
        f_t: float, 
        sigma: float
        ) -> float:
        """
        It calculates the new a0 value with the following formula:
        TODO: why 0.2?
        a0 = a0 + f_t * 0.2 + gaussian_noise(mean = 0, variance = sigma)
        Input:
            f_t: the frequency of the magnetic field
            sigma: the variance of the gaussian noise
        Output:
            a0: the new a0 value
        """
        return self.a0 + f_t*0.2 + np.random.normal(0, sigma, 1)[0]

    def simulate(
        self, 
        i: int,
        t, 
        states: Tuple[float, float]
        ) -> Tuple[float, float]:
        """
        It calculates the derivative for the current state, but it does not update the state!
        Input:
            t: the current time(not used in this function)
            states: the current state of the robot, x and y position(again not used in this function)
        Output:
            state_prime: the derivative of the state, x and y velocity
        """
        print("simulate for agent {}".format(i))
        # currently we assume that all the robots are identical
        #if i != 0:
        #    return self.state_primes[i]
        f_t = self.current_action[0]
        alpha_t = self.current_action[1]    
        
        mu, sigma = 0, self.noise_var # mean and standard deviation

        # get the value of a0 -- either constant or with model mismatch
        a0 = self.a0
        if self.is_mismatched:
            # TODO: why sigma/4?
            a0 = self.a0_linear(f_t, sigma/4)
            dx = a0 * f_t  * np.cos(alpha_t + 0.1) + 0.2
            dy = a0 * f_t  * np.sin(alpha_t + 0.1) - 0.1
        else:
            dx = a0 * f_t  * np.cos(alpha_t) 
            dy = a0 * f_t  * np.sin(alpha_t) 

        self.state_primes = [
            np.array(
                [
                    dx + np.random.normal(mu, sigma, 1)[0], 
                    dy + np.random.normal(mu, sigma, 1)[0]
                ]
            )
            for _ in range(self.number_of_agents)
        ]
        return self.state_primes[i]

    def scipy_runge_kutta(
        self, 
        fun, 
        y0, 
        t0 = 0, 
        t_bound = 10
        ) -> RK45:
        """
        It creates an integrator for the simulator
        Input:
            fun: the function to integrate
            y0: the initial state
            t0: the initial time
            t_bound: the final time
        Output:
            integrator: the integrator for the simulator
        """
        return RK45(
            fun, 
            t0, 
            y0, 
            t_bound, 
            rtol = self.time_span/self.number_iterations, 
            atol = 1e-4)

    def get_state(self, i: int):
        # The last state of the robot.
        return self.last_states[i]

