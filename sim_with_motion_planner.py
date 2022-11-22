import numpy as np
import enum

from main_2d import execute_idle_action, execute_learn_action
import Learning_module_2d as GP
from motion_planner.motion_planner import RRT

from MR_env import MR_Env
from utils import find_alpha_corrected
from motion_planner.utils import get_distance

# enum for choosing between controller types
class ControllerType(enum.Enum):
    P = 0
    MPC = 1


SIMULATION_FREQ_HZ = 30
MAGNETIC_FIELD_FREQ = 2
ACCEPTED_DISTANCE = 0.2
NOISE = 0.0
CONTROLLER_TYPE = ControllerType.P


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

def p_controller(init_state, path, gp_sim, env, obstacles):
    executed_path = [init_state]
    curr_state = init_state
    for temp_aim in path:
        while get_distance(curr_state[0], curr_state[1], temp_aim[0], temp_aim[1]) > ACCEPTED_DISTANCE:
            # Just a simple P controller
            difference = np.array(temp_aim) - curr_state
            print(difference)
            P = 0.1
            desired_speed = P * difference
            print(desired_speed)
            # get the alpha value for this speed
            alpha_and_f_d, muX, muY, sigX, sigY = find_alpha_corrected(desired_speed, gp_sim)
            alpha = alpha_and_f_d[0]
            f_t = MAGNETIC_FIELD_FREQ
            
            print("-----------------------------")
            print(f"past state: {curr_state}")
            print(f"desired speed: {desired_speed}")
            print(f"speed's angle", np.arctan2(desired_speed[1], desired_speed[0]))
            #alpha = np.arctan2(desired_speed[1], desired_speed[0])
            print(f"alpha: {alpha}")
            print(f"f_t: {f_t}")
            print(f"muX: {muX}")
            print(f"muY: {muY}")
            print(f"sigX: {sigX}")
            print(f"sigY: {sigY}")

            env.step(f_t, alpha)
            past_state = curr_state
            curr_state = env.last_pos
            print("curr_state: ", curr_state)
            print("state difference: ", np.array(curr_state) - np.array(past_state))
            print("-----------------------------")
            executed_path.append(curr_state)

            #plot(obstacles, [start_x, start_y], [goal_x, goal_y], to_be_followed_path, executed_path)
    plot(obstacles, path[0], path[-1], path, executed_path)

def mpc_controller(init_state, path, gp_sim, env, obstacles):
    """
    Implements model predictive controller
    """
    pass



def main():
    gp_sim = GP.LearningModule()

    execute_idle_action(gp_sim, noise_var = NOISE)
    a0_sim = execute_learn_action(gp_sim, noise_var = NOISE, plot = True)

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
    max_iter = 5000
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
        noise_var = NOISE,
        a0 = a0_sim,
        is_mismatched = True)

    if CONTROLLER_TYPE == ControllerType.P:
        p_controller(curr_state, to_be_followed_path, gp_sim, env, obstacles)

    
        





if __name__ == "__main__":
    main()





