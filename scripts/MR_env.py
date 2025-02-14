#!/usr/bin/python
#-*- coding: utf-8 -*-

import numpy as np

from gym import Env, spaces

from MR_viewer import Viewer
from MR_simulator import Simulator
from plot_utils import save_frames_as_gif

from typing import Tuple, List
"""
TODO :
- Reward Function w/logical specs?
- 

"""

REWARD_SUCCESS = 100
REWARD_FAILURE = -100
REWARD_STEP = -0.1

class MR_Env(Env):
    def __init__(
        self, 
        type = 'continuous', 
        action_dim = 2,
        number_of_agents = None
        ):

        self.type = type
        self.action_dim = action_dim
        self.number_of_agents = number_of_agents

        # assert type == 'continuous' or type == 'discrete', 'type must be continuous or discrete'
        # assert action_dim > 0 and action_dim <=2, 'action_dim must be 1 or 2'
        
        self.action_spaces = [
            spaces.Box(
                low = np.array([0, 0]), 
                high = np.array([20, np.pi*2])
            )
            for _ in range(self.number_of_agents)
        ]
        self.observation_spaces = [
            spaces.Box(
                low = np.array([-5000, -5000, -5000, -5000, 0]),
                high = np.array([5000, 5000, 5000, 5000, 80000])
            )
            for _ in range(self.number_of_agents)
        ]
        self.init_spaces = [
            spaces.Box(
                low = np.array([100, 100]),
                high = np.array([120, 120])
            )
            for _ in range(self.number_of_agents)
        ]
        self.init_goal_spaces = [
            spaces.Box(
                low = np.array([-31, -31]),
                high = np.array([-32, -32])
            )
            for _ in range(self.number_of_agents)
        ]
        self.borders = [ 
            [-510, 510], 
            [-510, -510], 
            [510, -510], 
            [510, 510]]

        self.simulator = Simulator(number_of_agents = self.number_of_agents)
        
        self.last_positions = [np.zeros(2) for _ in range(self.number_of_agents)]
        # TODO: Why init_goal and not just goal?
        self.init_goals = [np.zeros(2) for _ in range(self.number_of_agents)]
        self.last_actions = [np.zeros(self.action_dim) for _ in range(self.number_of_agents)]
        
        self.number_loop = 0  # loops in the screen -> used to plot
        self.counter = 0
        self.max_timesteps = 50
        self.min_dist2goal = 30
        
        self.viewer = None
        self.MR_data = None
        self.name_experiment = None
        self.state_primes = [None for _ in range(self.number_of_agents)]

    def step(
        self, 
        f_t: float,
        alpha_t: float
        ) -> bool:
        """
        This method is used to take a step in the environment and to update the states
        Input:
            f_t: frequency of the magnetic field
            alpha_t: yaw angle of the robot
        Output:
            done (boolean): whether it’s time to reset the environment again.
        """
        # According to the action stace a different kind of action is selected
        self.counter += 1
        states = self.simulator.step(f_t, alpha_t)
        # TODO: Implement this method
        self.state_primes = self.simulator.state_primes

        # convert simulator states into observable states
        obss = [self.convert_state_to_observable(state, self.init_goals[j]) for j, state in enumerate(states)]
        
        self.last_positions = [[state[0], state[1]] for state in states]
        self.last_actions = np.array([f_t, alpha_t])

        if self.MR_data is not None:
            # TODO: why 10?
            rew = 10 # self.calculate_reward(obs=obs)
            # TODO Need to update this method!
            self.MR_data.new_transition(states, obss, self.last_action, rew)
        
        # TODO: Implement this method
        done = self.should_end(obss)
        return done
    
    def convert_state_to_observable(
        self, 
        state: Tuple[float, float], 
        goal_loc: Tuple[float, float]
        ) -> Tuple[float, float, float, float, float]:
        """
        This method generates the features used to build the reward function, 
            converts the current state to the observable state
        Input:
            state (tuple): current state of the robot
            goal_loc (tuple): goal position
        Returns:
            obs (object): an environment-specific object representing 
                your observation of the environment: [x, y, goal_x, goal_y, distance to target]
        """
        x, y, goal_x, goal_y  = state[0], state[1], goal_loc[0], goal_loc[1]

        d = np.linalg.norm([goal_x - x, goal_y - y])
        return np.array([x, y, goal_x, goal_y, d])

    def calculate_reward(
        self, 
        obs: Tuple[float, float, float, float, float]
        ) -> float:
        """
        This method calculates the reward based on the observable state
        TODO: WHY?
        Input:
            obs (tuple): observable state
        Output:
            rew (float): amount of reward achieved by the previous action
        """
        d = obs[4]
        if d < self.min_dist2goal:
            print("\n ############ Got there ########")
            return REWARD_SUCCESS
        elif not self.observation_space.contains(obs) or self.counter > self.max_timesteps:
            return REWARD_FAILURE
        else:
            return REWARD_STEP # self.min_dist2goal/d

    # TODO: Implement this method, currently only looking to the first agent!!!!!
    def should_end(
        self, 
        obss: List[Tuple[float, float, float, float, float]]
        ) -> bool:
        """
        This method finds out whether we are at the end of episode
        Input:
            obs (tuple): observable state
        Output:
            done (boolean): whether it’s time to reset the environment again.
        """
        agent_num = 0
        d = obss[agent_num][4]
        # smashed into a wall or reached to the max timesteps
        if not self.observation_spaces[agent_num].contains(obss[agent_num]) or self.counter > self.max_timesteps:
            if self.viewer is not None:
                self.viewer.end_episode()
            if self.MR_data is not None:
                if self.MR_data.iterations > 0:
                    self.MR_data.save_experiment(self.name_experiment)
            return True
        elif d < self.min_dist2goal:
            return True
        else:
            return False

    # TODO: Implement this method
    # TODO: What is this????
    def set_goal(self,init):
        # self.init_goal = self.init_goal_space.sample()
        # while np.linalg.norm( self.init_goal - init ) < self.min_dist2goal :
        #     self.init_goal = self.init_space.sample()
        #     # print("uh")
        return self.init_goals

    def reset(
        self, 
        init: List[Tuple[float, float]] = None, 
        noise_var = 1, 
        a0 = 1, 
        is_mismatched = False
        ) -> Tuple[float, float, float, float, float]:

        if init is None: 
            init = self.init_space.sample()
            
        print("BE CAREFUL THAT THIS METHOD MAY NOT SUPPORT EVERY MULTI_ROBOT ENVIRONMENT")
        #print("starting positions")
        #for i in range(self.number_of_agents):
        #    print(init[i].shape)
        # TODO: Check this function?
        self.set_goal(init)
        
        self.simulator.noise_var = noise_var
        self.simulator.a0 = a0
        self.simulator.reset_start_pos(init)
        self.init_goals = [self.init_spaces[i].sample() for i in range(self.number_of_agents)]
        self.simulator.is_mismatched = is_mismatched
        
        self.last_positions = init
        self.counter = 0
        # print('Reseting position')
        # print( "goal_loc ", self.goal_loc ,"init_pos",self.last_pos)
        # TODO: Implement this method
        states = []
        for i in range(self.number_of_agents):
            #print("getting state for agent ", i)
            states.append(self.simulator.get_state(i))
        if self.MR_data is not None:
            if self.MR_data.iterations > 0:
                self.MR_data.save_experiment(self.name_experiment)
            # TODO: Update this method
            self.MR_data.new_iter(
                                states, 
                                [self.convert_state_to_observable(state, self.init_goals[j]) for j, state in enumerate(states)], 
                                np.zeros(len(self.last_action)), 
                                np.array([0])
                                )
        if self.viewer is not None:
            self.viewer.end_episode()
        return [self.convert_state_to_observable(state, self.init_goals[j]) for j, state in enumerate(states)]

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer()
            self.viewer.plot_boundary(self.borders)
            
        if 1 > self.number_loop: # ??
            self.viewer.end_episode()
            self.viewer.plot_position(self.last_pos[0], self.last_pos[1])
            self.viewer.restart_plot()
            self.number_loop += 1
        else:
            self.viewer.plot_goal(self.init_goal, 2)
            self.viewer.plot_position(self.last_pos[0], self.last_pos[1])
            self.viewer.end_episode()
            self.viewer.restart_plot()

    def close(self, ):
        self.viewer.freeze_scream()


if __name__ == '__main__':
    frames = []
    mode = 'normal' # mode: 'normal', 'joystick'

    if mode == 'normal':
        env = MR_Env()
        episode_action_obs = []
        for i_episode in range(1):
            observation = env.reset()
            for t in range(20):
                frames.append(env.render())
                # env.render()
                action = np.array([20, np.pi/4]) # -2*np.pi/(i_episode+1)
                done = env.step(action[0], action[1])
                
                if done:
                    print("Episode finished after {} timesteps".format(t + 1))
                    break
        env.close()
        save_frames_as_gif(frames)
        print("######### DONE ########")

    elif mode == 'joystick':

        print('joystick not implemented')

    else:
        print("mode should be  'normal' or 'joystick'") 


