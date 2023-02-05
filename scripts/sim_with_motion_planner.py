import numpy as np
from consts import *
import logging
import cv2

from main_2d import execute_idle_action, execute_learn_action
import Learning_module_2d as GP
from motion_planner.motion_planner import RRT
from mpc import MPC

from MR_logger import mr_logger as log
from MR_env import MR_Env
from utils import find_alpha_corrected
from motion_planner.utils import get_distance

from vision.find_ellips import execute_cell_separation_from_img


def plot(obstacles, start_points, goal_points, to_be_followed_paths, executed_paths, curr_pose = None):
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    _, ax = plt.subplots()
    for obstacle in obstacles:
        # plot as polygons
        ax.add_patch(patches.Polygon(obstacle, closed=True, fill=True, color='gray'))
        #pass
    for i in range(NUMBER_OF_AGENTS):
        ax.plot(start_points[i][0], start_points[i][1], 'bo')
        ax.plot(goal_points[i][0], goal_points[i][1], 'go')
        to_be_followed_path = np.array(to_be_followed_paths[i])
        ax.plot(to_be_followed_path[:,0], to_be_followed_path[:,1], 'b')
        executed_path = np.array(executed_paths[i])
        ax.plot(executed_path[:,0], executed_path[:,1], 'r')
    # put boundaries
    """
    if LOGGER_LEVEL == logging.DEBUG:
        ax.set_xlim([curr_pose[0] - 1, curr_pose[0] + 1])
        ax.set_ylim([curr_pose[1] - 1, curr_pose[1] + 1])
    """
    if False:
        pass
    else:
        ax.set_xlim([ENV_MIN_X, ENV_MIN_X + ENV_WIDTH])
        ax.set_ylim([ENV_MIN_Y, ENV_MIN_Y + ENV_HEIGHT])
    plt.show()

def plot_only_obstacles(obstacles):
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    _, ax = plt.subplots()
    for obstacle in obstacles:
        # plot as polygons
        ax.add_patch(patches.Polygon(obstacle, closed=True, fill=True, color='gray'))
        #pass
    # put boundaries
    ax.set_xlim([ENV_MIN_X, ENV_MIN_X + ENV_WIDTH])
    ax.set_ylim([ENV_MIN_Y, ENV_MIN_Y + ENV_HEIGHT])
    plt.show()

def controller(init_states, paths, gp_sim, env, obstacles, verbose=False, controller_type=None):
    executed_paths = []
    for i in range(NUMBER_OF_AGENTS):
        executed_paths.append([init_states[i]])
    curr_states = []
    for i in range(NUMBER_OF_AGENTS):
        curr_states.append(init_states[i])

    # the first point is the start point, so skipping it!
    path_indexes = [1] * NUMBER_OF_AGENTS
    reached_agent_count = 0
    # while not all the agents reached their goal!!!!!!!!!
    number_of_agents_placeholder = 1
    import time
    while True:
        start = time.time()
        if reached_agent_count == number_of_agents_placeholder:
            break
        for i in range(NUMBER_OF_AGENTS):
            if i == 0:
                continue
            curr_state = env.last_positions[i]
            executed_paths[i].append(curr_state)

        for i in range(number_of_agents_placeholder):
            if len(paths[i]) == path_indexes[i]:
                # agent i reached its goal
                reached_agent_count += 1
                log.warning(f"agent {i} reached its goal")
                continue
            rest_path = paths[i][path_indexes[i]:] if path_indexes[i] != 0 else paths[i]
            mpc = MPC(
                path = rest_path, 
                curr_position = curr_states[i],
                a0 = gp_sim.a0,
                )
            try:
                f, alpha = mpc.run()
            except:
                log.error(f"MPC failed for agent {i}")
                plot(obstacles, paths[i][0], paths[i][-1], paths[i], executed_paths[i], curr_pose = curr_states[i])
                return

            f = f[1]
            alpha = alpha[1]
            desired_speed = np.array([gp_sim.a0 * f * np.cos(alpha), gp_sim.a0 * f * np.sin(alpha)])

            alpha_and_f_d, muX, muY, sigX, sigY = find_alpha_corrected(desired_speed, gp_sim)
            #log.debug(f"alpha before corrected: {alpha} for agent {i}")
            alpha = alpha_and_f_d[0]
            #log.debug(f"alpha after corrected: {alpha} for agent {i}")
            #log.debug(f"f_t: {f} for agent {i}")

            log.debug(f"desired speed: {desired_speed} for agent {i}")
            log.debug(f"past state: {curr_states[i]} for agent {i}")

            env.step(f, alpha)
            curr_states[i] = env.last_positions[i]
            log.debug(f"current state: {curr_states[i]} for agent {i}")
            log.debug(f"next_goal: {paths[i][path_indexes[i]]} for agent {i}\n")

            executed_paths[i].append(curr_states[i])

            if LOGGER_LEVEL == logging.DEBUG:
                """
                print(paths)
                print(paths[0])
                print([paths[j][0] for j in range(NUMBER_OF_AGENTS)])
                print([paths[j][-1] for j in range(NUMBER_OF_AGENTS)])
                print(paths[i])
                print(executed_paths[i])
                print(curr_states[i])
                """
                #plot(obstacles, [paths[j][0] for j in range(NUMBER_OF_AGENTS)], [paths[j][-1] for j in range(NUMBER_OF_AGENTS)], paths, executed_paths, curr_pose = curr_states[i])
            
            log.debug(f"distance from goal: {get_distance(curr_states[i][0], curr_states[i][1], paths[i][path_indexes[i]][0], paths[i][path_indexes[i]][1])} for agent {i}")
            if get_distance(curr_states[i][0], curr_states[i][1], paths[i][path_indexes[i]][0], paths[i][path_indexes[i]][1]) < ACCEPTED_DISTANCE:
                log.warning(f"agent {i} reached one of the points in the path, {path_indexes[i]}/{len(paths[i])}")
                path_indexes[i] += 1
                #plot(obstacles, paths[0], paths[-1], paths, executed_paths)

            log.debug("-----------------------------")
        end = time.time()
        print(f"iteration time: {end - start}")

    # NEED TO UPDATE THIS !!!! TODO
    plot(obstacles, paths[0], paths[-1], paths, executed_paths)

def main():
    global OBSTACLES
    gp_sim = GP.LearningModule()

    execute_idle_action(gp_sim, noise_var = NOISE)
    a0_sim = execute_learn_action(gp_sim, noise_var = NOISE, plot = True)
    ### We learned the noise and a0, so now it is time to create the test environment!

    log.warning("Analyzing the environment using image")
    img = cv2.imread("vision/images/clustered_frame_23_jan.png")
    all_obstacles = execute_cell_separation_from_img(img)

    #print("first obstacle in the all obstacles:", all_obstacles[0])
    #print("first obstacle in the const:", OBSTACLES[0])
    OBSTACLES = all_obstacles
    plot_only_obstacles(all_obstacles)
    log.warning("Number of obstacles found: " + str(len(all_obstacles)))

    paths = []
    for i in range(NUMBER_OF_AGENTS):
        rrt = RRT(
            ROBOTS_START_X[i], 
            ROBOTS_START_Y[i], 
            ROBOTS_GOAL_X[i], 
            ROBOTS_GOAL_Y[i], 
            RRT_STEP_SIZE, 
            RRT_REWIRE_DISTANCE,
            RRT_MAX_ITER, 
            ENV_MIN_X, 
            ENV_MIN_Y, 
            ENV_WIDTH,
            ENV_HEIGHT, 
            OBSTACLES,
            img)
    
        paths.append(rrt.RRTStar(plot = False, rewire = False, repeat = True))

    # save paths for debugging
    import pickle
    with open("paths2.pkl", "wb") as f:
        pickle.dump(paths, f)
    
    #import pickle
    #with open("paths.pkl", "rb") as f:
    #    paths = pickle.load(f)
    """
    # used for debugging
    paths = [
        [
            [-2.5, 7.5],
            [-2.5, 6.0],
            [-2.5, 4.5],
        ],
        [
            [-1.5, 7.5],
            [-1.5, 6.0],
            [-1.5, 4.5],
        ],
    ]
    """
    
    curr_states = []
    for i in range(NUMBER_OF_AGENTS):
        curr_states.append(np.array([ROBOTS_START_X[i], ROBOTS_START_Y[i]]))
    log.warning("Creating simulation environment...")
    env = MR_Env(number_of_agents = NUMBER_OF_AGENTS)
    env.reset(
        init = curr_states,
        noise_var = NOISE,
        a0 = a0_sim,
        is_mismatched = False)

    log.warning("Starting controller...")
    controller(curr_states, paths, gp_sim, env, OBSTACLES, verbose = False, controller_type = CONTROLLER_TYPE)


if __name__ == "__main__":
    main()





