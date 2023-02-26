import matplotlib.pyplot as plt
import numpy as np
import copy
import random
import os
import json

from utils import generate_random_frequencies, generate_random_positions, generate_random_frequencies2N
from const import *
from time import time

frequencies = [
    [4.0215, 3.8835, 3.561, 3.699, 3.5925], # freq 1
    [5.9475, 6.735, 6.5565, 4.83975, 5.13225], # freq 5
    [8.8935, 9.225, 9.912, 8.53875, 7.008], # freq 10
    [12.5745, 10.848, 10.6785, 11.679, 8.54475], # freq 20
    [16.218, 12.5205, 13.0635, 17.63475, 9.279], # freq 30
]

#frequencies = generate_random_frequencies2N()
current_positions = generate_random_positions()
#goal_positions = generate_random_positions()

goal_positions = [
    [40, 30],
    [60, 30],
    [66.49, 49.02],
    [50, 55.51],
    [33.51, 49.02]
]
plotted_alphas = []
application_times = []
total_distances = []

# flatten the current positions and the goal positions
current_positions_flat = [item for sublist in current_positions for item in sublist]
goal_positions_flat = [item for sublist in goal_positions for item in sublist]
diff = np.array(goal_positions_flat) - np.array(current_positions_flat)

"""
# take the inverse matrix of the frequencies
inverse_frequencies = np.linalg.inv(np.array(frequencies))
command = inverse_frequencies * diff

print('frequencies:')
print(frequencies)
print("command:")
print(command)
exit()
"""

# open a folder for each experiment
folder_name = f'experiments/experiment_{int(time())}'
os.mkdir(folder_name)
os.mkdir(f'{folder_name}/images')

# save experiment parameters to a json file
with open(f'{folder_name}/experiment_parameters.json', 'w') as f:
    json.dump({
        'frequencies': frequencies,
        'current_positions': current_positions,
        'goal_positions': goal_positions
    }, f)

cm = plt.get_cmap('gist_rainbow')
colors_for_plots = [cm(1.*i/NUMBER_OF_AGENTS) for i in range(NUMBER_OF_AGENTS)]

#plt.figure()

# plot the initial positions
current_positions_x = [position[0] for position in current_positions]
current_positions_y = [position[1] for position in current_positions]
'''
for i in range(NUMBER_OF_AGENTS):
    plt.scatter(current_positions_x[i], current_positions_y[i], c = colors_for_plots[i])
'''
# plot the goal positions
goal_positions_x = [position[0] for position in goal_positions]
goal_positions_y = [position[1] for position in goal_positions]
'''
for i in range(NUMBER_OF_AGENTS):
    plt.scatter(goal_positions_x[i], goal_positions_y[i], c = [np.array(colors_for_plots[i]) - np.array([0, 0, 0, 0.8])])
plt.savefig(f'{folder_name}/images/0.png')
'''

movements = []
for i in range(NUMBER_OF_AGENTS):
    movements.append([])
    movements[i].append(np.array(current_positions[i]))

def main():
    figure_index = 1
    step_number = 0
    last_important_positions = copy.deepcopy(current_positions)
    while True:
        for f in frequencies:
            t1 = -1 * np.sum((np.array([position[0] for position in current_positions]) - np.array([position[0] for position in goal_positions])) * np.array(f))
            t2 = -1 * np.sum((np.array([position[1] for position in current_positions]) - np.array([position[1] for position in goal_positions])) * np.array(f))

            norm_f = np.linalg.norm(f)

            t1 /= (norm_f ** 2)
            t2 /= (norm_f ** 2)

            alpha = np.arctan2(t2, t1)
            plotted_alphas.append(alpha)
            application_time = np.sqrt(t1 ** 2 + t2 ** 2)
            application_times.append(application_time)

            # apply the action
            past_positions = copy.deepcopy(current_positions)
            for i in range(NUMBER_OF_AGENTS):
                current_positions[i] += f[i] * application_time * np.array([np.cos(alpha), np.sin(alpha)])
                movements[i].append(copy.deepcopy(current_positions[i]))

            '''  
            # add the last action to the plot
            for i in range(NUMBER_OF_AGENTS):
                plt.plot([past_positions[i][0], current_positions[i][0]], [past_positions[i][1], current_positions[i][1]], c = colors_for_plots[i])
            '''    
            step_number += 1
        
            """
            # calculate the total distance from the last important positions
            total_distance = 0
            for i in range(NUMBER_OF_AGENTS):
                total_distance += np.linalg.norm(np.array(current_positions[i]) - np.array(last_important_positions[i]))
            # if the total distance is significant, we will save the plot
            if total_distance > (NUMBER_OF_AGENTS * 5):
                # set the title of the plot as the step number
                plt.title(f'Step {step_number}')
                plt.savefig(f'{folder_name}/images/{figure_index}.png')
                figure_index += 1
                last_important_positions = copy.deepcopy(current_positions)
            """
        
        # get the total distance between the agents and the goal
        total_distance = 0
        for i in range(NUMBER_OF_AGENTS):
            total_distance += np.linalg.norm(np.array(current_positions[i]) - np.array(goal_positions[i]))

        total_distances.append(total_distance)
        print(total_distance)
        if total_distance < 0.1:
            break


    import pickle
    with open(f'movements.pickle', 'wb') as f:
        pickle.dump(movements, f)
    with open(f'plotted_alphas.pickle', 'wb') as f:
        pickle.dump(plotted_alphas, f)
    with open(f'application_times.pickle', 'wb') as f:
        pickle.dump(application_times, f)
    with open(f'total_distances.pickle', 'wb') as f:
        pickle.dump(total_distances, f)

    # we will convert the images to a video using the following command:
    # ffmpeg -framerate 30 -i "%d.png" exp.mp4
    #os.system(f'ffmpeg -framerate 30 -i "{folder_name}/images/%d.png" {folder_name}/exp.mp4')

if __name__ == "__main__":
    main()