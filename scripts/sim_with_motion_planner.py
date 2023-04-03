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

import rrt_fast


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
        # print executed_paths[i] as a set of points, by connecting them with lines
        executed_path = np.array(executed_paths[i])
        # plot them two by two
        for j in range(len(executed_path) - 1):
            if i == 0:
                ax.plot([executed_path[j,0], executed_path[j+1,0]], [executed_path[j,1], executed_path[j+1,1]], 'r')
            if i == 1:
                ax.plot([executed_path[j,0], executed_path[j+1,0]], [executed_path[j,1], executed_path[j+1,1]], 'g')
            if i == 2:
                ax.plot([executed_path[j,0], executed_path[j+1,0]], [executed_path[j,1], executed_path[j+1,1]], 'y')
            log.debug("plotting line from {} to {}".format(executed_path[j], executed_path[j+1]))

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
    frequencies = [
        [11, 5, 3],
        [4, 2, 1],
        [3, 1, 0.2]
    ]
    init_positions = [
        [100, 100],
        [200, 200],
        [200, 100]
    ]
    current_positions = [
        [100, 100],
        [200, 200],
        [200, 100]
    ]
    goal_positions = [
        [200, -200],
        [-100, 300],
        [-100, 100]
    ]
    while True:
        #if reached_agent_count == number_of_agents_placeholder:
        #    break
        for _ in range(4000):
                
            for f in frequencies:
                t1 = -1
                t2 = -1
                t1 *= np.sum((np.array([position[0] for position in current_positions]) - np.array([position[0] for position in goal_positions])) * np.array(f))
                t2 *= np.sum((np.array([position[1] for position in current_positions]) - np.array([position[1] for position in goal_positions])) * np.array(f))
                #t1 *= np.sum((np.array(current_positions[::0]) - np.array(goal_positions[::0])) * np.array(f))
                #t2 *= np.sum(np.array(current_positions[::1] - np.array(goal_positions[::1])) * np.array(f))

                norm_f = np.linalg.norm(f)

                t1 /= (norm_f ** 2)
                t2 /= (norm_f ** 2)

                alpha = np.arctan2(t2, t1)
                application_time = np.sqrt(t1 ** 2 + t2 ** 2)

                # I am not doing alpha correction here, because it wants desired speed
                # but which agent's desired speed? I don't know!

                # apply the action
                for i in range(len(current_positions)):
                    current_positions[i] += f[i] * application_time * np.array([np.cos(alpha), np.sin(alpha)])

                # add the new position to the executed path
                for i in range(len(current_positions)):
                    executed_paths[i].append([current_positions[i][0], current_positions[i][1]])
                    
                
                log.debug("current_positions", current_positions)
                log.debug("goal_positions", goal_positions)
                log.debug("executed_paths", executed_paths)

        plot(
            [], 
            init_positions, 
            goal_positions, [
            [init_positions[0], goal_positions[0]],
            [init_positions[1], goal_positions[1]],
            [init_positions[2], goal_positions[2]]
        ], executed_paths)

        continue

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

    # NEED TO UPDATE THIS !!!! TODO
    plot(obstacles, paths[0], paths[-1], paths, executed_paths)

def main():
    global OBSTACLES
    #gp_sim = GP.LearningModule()

    #execute_idle_action(gp_sim, noise_var = NOISE)
    #a0_sim = execute_learn_action(gp_sim, noise_var = NOISE, plot = True)
    ### We learned the noise and a0, so now it is time to create the test environment!

    #log.warning("Analyzing the environment using image")
    img = cv2.imread("vision/images/clustered_frame_23_jan.png")
    #all_obstacles = execute_cell_separation_from_img(img)

    #print("first obstacle in the all obstacles:", all_obstacles[0])
    #print("first obstacle in the const:", OBSTACLES[0])
    #OBSTACLES = all_obstacles
    #plot_only_obstacles(all_obstacles)
    #log.warning("Number of obstacles found: " + str(len(all_obstacles)))
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    height, width = img.shape[:2]

    img_array = np.zeros((height, width, 3), np.uint8)

    for i in range(height):
        for j in range(width):
            img_array[i][j][0] = img[i][j][0]
            img_array[i][j][1] = img[i][j][1]
            img_array[i][j][2] = img[i][j][2]

    """
    import time
    total_time = 0
    for i in range(20):
        start_time = time.time()
        paths = []
        a = rrt_fast.rrt(
            NUMBER_OF_AGENTS,
            ROBOTS_START_X,
            ROBOTS_START_Y,
            ROBOTS_GOAL_X,
            ROBOTS_GOAL_Y,
            RRT_STEP_SIZE,
            RRT_REWIRE_DISTANCE,
            RRT_MAX_ITER,
            ENV_MIN_X,
            ENV_MIN_Y,
            ENV_WIDTH,
            ENV_HEIGHT,
            SPEEDS_FOR_FREQ,
            list(img_array),
        )
        print("time taken:", time.time() - start_time)
        total_time += time.time() - start_time
    
    print("average time taken:", total_time/20)
    exit(1)
    """
    # take the matrix inverse of speed_for_freq
    inversed_frequencies = np.array(SPEEDS_FOR_FREQ)
    # decrease the array into a num_agents * num_agents matrix
    inversed_frequencies = inversed_frequencies[:NUMBER_OF_AGENTS, :NUMBER_OF_AGENTS]
    inversed_frequencies = np.linalg.inv(inversed_frequencies)

    a = rrt_fast.rrt(
            NUMBER_OF_AGENTS,
            ROBOTS_START_X,
            ROBOTS_START_Y,
            ROBOTS_GOAL_X,
            ROBOTS_GOAL_Y,
            RRT_STEP_SIZE,
            RRT_REWIRE_DISTANCE,
            RRT_MAX_ITER,
            ENV_MIN_X,
            ENV_MIN_Y,
            ENV_WIDTH,
            ENV_HEIGHT,
            SPEEDS_FOR_FREQ,
            list(img_array),
            inversed_frequencies
        )
    
    # the list a is elements of the form (configuration, parent id)
    # configuration is a 2*Number_of_agents array
    
    #print("a:", a)

    import matplotlib.pyplot as plt
    import matplotlib
    #print("a:", a)


    for element in a:
        configuration = element[0]
        parent_id = element[1]
        if parent_id == -1:
            continue
        for i in range(NUMBER_OF_AGENTS):
            #print("printing line")
            #print("point 1:", configuration[i])
            #print("point 2:", a[parent_id][0][i])
            # draw a line between the two points
            # pick color from rainbow
            plt.plot([configuration[i][0], a[parent_id][0][i][0]], [configuration[i][1], a[parent_id][0][i][1]], color = matplotlib.colors.hsv_to_rgb([i/NUMBER_OF_AGENTS, 1, 1]))

    # plot start and goal positions
    for i in range(NUMBER_OF_AGENTS):
        plt.plot(ROBOTS_START_X[i], ROBOTS_START_Y[i], 'bo')
        plt.plot(ROBOTS_GOAL_X[i], ROBOTS_GOAL_Y[i], 'go')

    # get ax object
    ax = plt.gca()
    # plot obstacles
    # iterate over image,
    # if the pixel is not black, make that coordinate gray
    for i in range(height):
        for j in range(width):
            if img[i][j][0] != 0 or img[i][j][1] != 0 or img[i][j][2] != 0:
                # make the pixel gray
                #ax.add_patch(matplotlib.patches.Rectangle((i, j), 1, 1, color = 'gray'))
                pass

    plt.show()
            
    """
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
    
        paths.append(rrt.RRTStar(plot = True, rewire = False, repeat = True))
    """
    exit(1)
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
    controller(curr_states, [], gp_sim, env, OBSTACLES, verbose = False, controller_type = CONTROLLER_TYPE)


if __name__ == "__main__":
    main()





