import pickle
import numpy as np
import matplotlib.pyplot as plt

ENV_MIN_X = -1
ENV_MAX_X = 1
ENV_MIN_Y = -1
ENV_MAX_Y = 1

with open('configurations.pickle', 'rb') as f:
    configurations = pickle.load(f)

# convert the dictionary into a numpy array
configurations_arr = []
for x in configurations:
    for y in configurations[x]:
        for x2 in configurations[x][y]:
            for y2 in configurations[x][y][x2]:
                configurations_arr.append([x, y, x2, y2])

configurations_arr = np.array(configurations_arr)


# open two subplots
fig1 = plt.figure()
ax1 = fig1.add_subplot(121)
ax1.set_xlim(ENV_MIN_X, ENV_MAX_X)
ax1.set_ylim(ENV_MIN_Y, ENV_MAX_Y)

ax2 = fig1.add_subplot(122)
ax2.set_xlim(ENV_MIN_X, ENV_MAX_X)
ax2.set_ylim(ENV_MIN_Y, ENV_MAX_Y)

def fake_distance(x1, y1, x2, y2):
    return (x1 - x2)**2 + (y1 - y2)**2

# add the onclick event to the figure 1
def onclick1(event):
    xdata = event.xdata
    ydata = event.ydata
    # find the closest point in the configurations array for the x and y, first agent
    distances = []
    for x, y, _, _ in configurations_arr:
        distances.append(fake_distance(x, y, xdata, ydata))
    distances = np.array(distances)
    closest_index = np.argmin(distances)
    closest_x, closest_y, _, _ = configurations_arr[closest_index]
    x2sy2s = []
    for x2 in configurations[closest_x][closest_y]:
        for y2 in configurations[closest_x][closest_y][x2]:
            x2sy2s.append([x2, y2])
    
    print("Closest point: ({}, {})".format(closest_x, closest_y))

    # update the figure 1 to show the closest point
    ax1.clear()
    ax1.scatter(closest_x, closest_y, c='b')
    ax1.set_xlim(ENV_MIN_X, ENV_MAX_X)
    ax1.set_ylim(ENV_MIN_Y, ENV_MAX_Y)
    
    plt.draw()
    
    # update the figure 2 to show the x2y2s
    ax2.clear()
    ax2.scatter(np.array(x2sy2s)[:, 0], np.array(x2sy2s)[:, 1], c='b')
    ax2.set_xlim(ENV_MIN_X, ENV_MAX_X)
    ax2.set_ylim(ENV_MIN_Y, ENV_MAX_Y)

    plt.draw()

# connect the onclick event to the figure 1
cid1 = fig1.canvas.mpl_connect('button_press_event', onclick1)

# plot the x and y coordinates of the first agent in the figure 1
# plot the x and y coordinates of the second agent in the figure 2

ax1.scatter(configurations_arr[:, 0], configurations_arr[:, 1], c='b')
ax2.scatter(configurations_arr[:, 2], configurations_arr[:, 3], c='b')
ax1.set_xlim(ENV_MIN_X, ENV_MAX_X)
ax1.set_ylim(ENV_MIN_Y, ENV_MAX_Y)
ax2.set_xlim(ENV_MIN_X, ENV_MAX_X)
ax2.set_ylim(ENV_MIN_Y, ENV_MAX_Y)

plt.show()

