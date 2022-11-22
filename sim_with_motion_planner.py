import numpy as np

from main_2d import execute_idle_action, execute_learn_action
import Learning_module_2d as GP
from motion_planner.motion_planner import RRT

from MR_env import MR_Env
from utils import find_alpha_corrected
from motion_planner.utils import get_distance

SIMULATION_FREQ_HZ = 30
MAGNETIC_FIELD_FREQ = 4
ACCEPTED_DISTANCE = 0.05

def plot(obstacles, start_point, goal_point, to_be_followed_path, executed_path):
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    fig, ax = plt.subplots()
    for obstacle in obstacles:
        ax.add_patch(
            patches.Rectangle(
                (obstacle[0], obstacle[1]),   # (x,y)
                obstacle[2],          # width
                obstacle[3],          # height
                fill = True
            )
        )

    ax.plot(start_point[0], start_point[1], 'bo')
    ax.plot(goal_point[0], goal_point[1], 'go')
    to_be_followed_path = np.array(to_be_followed_path)
    ax.plot(to_be_followed_path[:,0], to_be_followed_path[:,1], 'b')
    executed_path = np.array(executed_path)
    ax.plot(executed_path[:,0], executed_path[:,1], 'r')
    plt.show()

def main():
    gp_sim = GP.LearningModule()

    execute_idle_action(gp_sim)
    a0_sim = execute_learn_action(gp_sim, plot = True)

    ### We learned the noise and a0, so now it is time to create the test environment!
    obstacles = [
        (-5, -5, 5, 5), 
        (5, 5, 5, 5),
        (0.1, 0.1, 4.8, 4.8)]
    start_x = -2.5
    start_y = 7.5
    goal_x = 7.5
    goal_y = -7.5
    step_size = 0.2
    max_iter = 2000
    env_min_x = -10
    env_min_y = -10
    env_width = 20
    env_height = 20
    rrt = RRT(
        start_x, 
        start_y, 
        goal_x, 
        goal_y, 
        step_size, 
        max_iter, 
        env_min_x, 
        env_min_y, 
        env_width,
        env_height, 
        obstacles)

    to_be_followed_path = rrt.RRTStar(plot = False, rewire = True, repeat = True)

    print(to_be_followed_path)

    curr_state = np.array([start_x, start_y])
    env = MR_Env()
    env.reset(
        init = curr_state,
        noise_var = 0.0,
        a0 = a0_sim,
        is_mismatched = True)

    executed_path = [curr_state]
    for temp_aim in to_be_followed_path:
        counter = 0
        while get_distance(curr_state[0], curr_state[1], temp_aim[0], temp_aim[1]) > ACCEPTED_DISTANCE:
            # Just a simple P controller
            difference = np.array(temp_aim) - curr_state
            print(difference)
            P = 0.001
            desired_speed = P * difference
            print(desired_speed)
            # get the alpha value for this speed
            alpha_and_f_d, _, _, _, _ = find_alpha_corrected(desired_speed, gp_sim)
            alpha = alpha_and_f_d[0]
            f_t = MAGNETIC_FIELD_FREQ
            print('-------------------------------')
            print(alpha)
            print(f_t)
            print('-------------------------------')

    
            env.step(f_t, alpha)
            curr_state = env.last_pos
            executed_path.append(curr_state)
            '''
            counter += 1
            if counter % 100 == 0:
                plot(obstacles, [start_x, start_y], [goal_x, goal_y], to_be_followed_path, executed_path)
            '''
    plot(obstacles, [start_x, start_y], [goal_x, goal_y], to_be_followed_path, executed_path)

        





if __name__ == "__main__":
    main()





