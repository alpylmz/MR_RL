import matplotlib.pyplot as plt
import numpy as np
import copy

NUMBER_OF_AGENTS = 3

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
plt.figure()
# plot the initial positions
for i in range(len(init_positions)):
    plt.plot(init_positions[i][0], init_positions[i][1], 'ro')

# plot the goal positions
for i in range(len(goal_positions)):
    plt.plot(goal_positions[i][0], goal_positions[i][1], 'go')
plt.savefig('init.png')
k = 0
while True:
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
        past_positions = copy.deepcopy(current_positions)
        for i in range(NUMBER_OF_AGENTS):
            current_positions[i] += f[i] * application_time * np.array([np.cos(alpha), np.sin(alpha)])
        
        # PLOT
        # add the last action to the plot
        for i in range(NUMBER_OF_AGENTS):
            if i == 0:
                plt.plot([past_positions[i][0], current_positions[i][0]], [past_positions[i][1], current_positions[i][1]], 'b')
            elif i == 1:
                plt.plot([past_positions[i][0], current_positions[i][0]], [past_positions[i][1], current_positions[i][1]], 'r')
            elif i == 2:
                plt.plot([past_positions[i][0], current_positions[i][0]], [past_positions[i][1], current_positions[i][1]], 'y')

            # show the plot, but do not delete the figure, we will add to it
            plt.savefig(f'plots/{k}.png', dpi=300)
            print(k)
            k += 1
        
    # check if we are done
    # since we are using floats, we need to check if they are close enough
    if np.allclose(current_positions, goal_positions, atol=0.01):
        break




