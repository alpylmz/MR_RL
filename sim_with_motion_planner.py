import numpy as np
import enum

from main_2d import execute_idle_action, execute_learn_action
import Learning_module_2d as GP
from motion_planner.motion_planner import RRT
from mpc import MPC

from MR_env import MR_Env
from utils import find_alpha_corrected
from motion_planner.utils import get_distance

# enum for choosing between controller types
class ControllerType(enum.Enum):
    P = 0
    MPC = 1


MAGNETIC_FIELD_FREQ = 2
# this can be much much smaller, but, beacuse we do not use magnetic field frequency, 
# we cannot control the speed, we can only control the direction of the speed
# therefore, we need to accept a bigger accepted distance for now
ACCEPTED_DISTANCE = 0.1
NOISE = 0.0
CONTROLLER_TYPE = ControllerType.MPC


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
                    curr_position = curr_state, 
                    w_u = 1, 
                    w_p = 10
                    )
                _, u = mpc.run()
                desired_speed = u[:2]

            # get the alpha value for this speed
            alpha_and_f_d, muX, muY, sigX, sigY = find_alpha_corrected(desired_speed, gp_sim)
            alpha = alpha_and_f_d[0]
            f_t = MAGNETIC_FIELD_FREQ
            
            if verbose:
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
            if verbose:
                print("curr_state: ", curr_state)
                print("state difference: ", np.array(curr_state) - np.array(past_state))
                print("-----------------------------")
            executed_path.append(curr_state)

    plot(obstacles, path[0], path[-1], path, executed_path)



def main():
    gp_sim = GP.LearningModule()

    execute_idle_action(gp_sim, noise_var = NOISE)
    a0_sim = execute_learn_action(gp_sim, noise_var = NOISE, plot = True)

    ### We learned the noise and a0, so now it is time to create the test environment!
    obstacles = [
        ([0.5, 0.5], [0.5, 1.5], [1.5, 1.5], [1.5, 0.5]),
        ([1.0, 0.0], [4.0, 0.0], [4.0, -5.0]),
        ([-2.0, 6.0], [-2.0, 2.0], [2.0, 4.0], [4.0, 6.0], [5.0, 7.0])
    ]
    start_x = -2.5
    start_y = 7.5
    goal_x = 7.5
    goal_y = -7.5
    step_size = 0.2
    rewire_distance = 5.0
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
        rewire_distance,
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

    controller(curr_state, to_be_followed_path, gp_sim, env, obstacles, verbose=False, controller_type=CONTROLLER_TYPE)


if __name__ == "__main__":
    main()





