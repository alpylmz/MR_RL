import pickle
from const import NUMBER_OF_AGENTS

with open('movements.pickle', 'rb') as f:
    movements = pickle.load(f)

# this is a 5x5x2 matrix, each row is a list of positions of an agent

import matplotlib.pyplot as plt

plt.figure()

# plot the initial positions with blue
current_positions_x = [position[0][0] for position in movements]
current_positions_y = [position[0][1] for position in movements]

for i in range(NUMBER_OF_AGENTS):
    plt.scatter(current_positions_x[i], current_positions_y[i], c = 'blue', s = 10)

# plot the final positions with red
goal_positions_x = [position[-1][0] for position in movements]
goal_positions_y = [position[-1][1] for position in movements]

for i in range(NUMBER_OF_AGENTS):
    plt.scatter(goal_positions_x[i], goal_positions_y[i], c = 'red', s = 10)

# colormap
cm = plt.get_cmap('gist_rainbow')
colors_for_plots = [cm(1.*i/10) for i in range(10)]
colors_for_plots = [colors_for_plots[i] for i in [2, 4, 5, 6, 8]]

# plot the paths
for i in range(NUMBER_OF_AGENTS):
    x = [position[0] for position in movements[i]]
    y = [position[1] for position in movements[i]]
    plt.plot(x, y, c = colors_for_plots[i], linewidth = 0.3)

plt.savefig('plot.png', dpi = 1200)
#plt.show()



