import numpy as np
from consts import *
from MR_RL_logger import mr_rl_logger as log
import logging

from main_2d import execute_idle_action, execute_learn_action
import Learning_module_2d as GP
from motion_planner.motion_planner import RRT
from mpc import MPC

from MR_env import MR_Env
from utils import find_alpha_corrected
from motion_planner.utils import get_distance

def plot(obstacles, start_point, goal_point, to_be_followed_path, executed_path):
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    _, ax = plt.subplots()
    for obstacle in obstacles:
        # plot as polygons
        ax.add_patch(patches.Polygon(obstacle, closed=True, fill=True, color='gray'))

    ax.plot(start_point[0], start_point[1], 'bo')
    ax.plot(goal_point[0], goal_point[1], 'go')
    to_be_followed_path = np.array(to_be_followed_path)
    ax.plot(to_be_followed_path[:,0], to_be_followed_path[:,1], 'b')
    executed_path = np.array(executed_path)
    ax.plot(executed_path[:,0], executed_path[:,1], 'r')
    # put boundaries
    ax.set_xlim([ENV_MIN_X, ENV_MIN_X + ENV_WIDTH])
    ax.set_ylim([ENV_MIN_Y, ENV_MIN_Y + ENV_HEIGHT])
    plt.show()

def controller(init_state, path, gp_sim, env, obstacles, verbose=False, controller_type=None):
    executed_path = [init_state]
    curr_state = init_state
    for i, temp_aim in enumerate(path):
        while get_distance(curr_state[0], curr_state[1], temp_aim[0], temp_aim[1]) > ACCEPTED_DISTANCE:
            if controller_type == ControllerType.P:
                difference = np.array(temp_aim) - curr_state
                P = 0.1
                desired_speed = P * difference
            elif controller_type == ControllerType.MPC:
                rest_path = path[i:] if i != 0 else path
                mpc = MPC(
                    path = rest_path, 
                    curr_position = curr_state
                    )
                _, u = mpc.run()
                desired_speed = u[:2]

                if verbose:
                    print(f"rest_path: {mpc.path}")

            # get the alpha value for this speed
            alpha_and_f_d, muX, muY, sigX, sigY = find_alpha_corrected(desired_speed, gp_sim)
            alpha = alpha_and_f_d[0]
            # this is bad solution, MPC should be able to handle this later TODO
            if np.sqrt(desired_speed[0]**2 + desired_speed[1]**2) < MPC_MIN_U_LIMIT:
                # scale the speed to the minimum speed
                desired_speed = desired_speed / np.linalg.norm(desired_speed) * MPC_MIN_U_LIMIT
            else:
                print("desired speed is ok")
            f_t = np.linalg.norm(desired_speed) / gp_sim.a0
            
            log.debug("-----------------------------")
            log.debug(f"past state: {curr_state}")
            log.debug(f"desired speed: {desired_speed}")
            log.debug(f"speed's angle {np.arctan2(desired_speed[1], desired_speed[0])}")
            #alpha = np.arctan2(desired_speed[1], desired_speed[0])
            log.debug(f"alpha: {alpha}")
            log.debug(f"f_t: {f_t}")
            log.debug(f"muX: {muX}")
            log.debug(f"muY: {muY}")
            log.debug(f"sigX: {sigX}")
            log.debug(f"sigY: {sigY}")


            env.step(f_t, alpha)
            past_state = curr_state
            curr_state = env.last_pos
            log.debug(f"curr_state: {curr_state}")
            log.debug(f"state difference: {np.array(curr_state) - np.array(past_state)}")
            log.debug("-----------------------------")

            executed_path.append(curr_state)

            if LOGGER_LEVEL == logging.DEBUG:
                plot(obstacles, path[0], path[-1], path, executed_path)

    plot(obstacles, path[0], path[-1], path, executed_path)



def main():
    gp_sim = GP.LearningModule()

    execute_idle_action(gp_sim, noise_var = NOISE)
    a0_sim = execute_learn_action(gp_sim, noise_var = NOISE, plot = True)

    ### We learned the noise and a0, so now it is time to create the test environment!
    rrt = RRT(
        ROBOT_START_X, 
        ROBOT_START_Y, 
        ROBOT_GOAL_X, 
        ROBOT_GOAL_Y, 
        RRT_STEP_SIZE, 
        RRT_REWIRE_DISTANCE,
        RRT_MAX_ITER, 
        ENV_MIN_X, 
        ENV_MIN_Y, 
        ENV_WIDTH,
        ENV_HEIGHT, 
        OBSTACLES)

    to_be_followed_path = rrt.RRTStar(plot = False, rewire = True, repeat = True)

    print(to_be_followed_path)

    curr_state = np.array([ROBOT_START_X, ROBOT_START_Y])
    env = MR_Env()
    env.reset(
        init = curr_state,
        noise_var = NOISE,
        a0 = a0_sim,
        is_mismatched = True)

    controller(curr_state, to_be_followed_path, gp_sim, env, OBSTACLES, verbose = False, controller_type = CONTROLLER_TYPE)


if __name__ == "__main__":
    main()





