import numpy as np
import matplotlib.pyplot as plt
import time

ENV_MIN_X = -10
ENV_MAX_X = 10
ENV_MIN_Y = -10
ENV_MAX_Y = 10

class Agent():
    def __init__(self, x, y, speed) -> None:
        self.x = x
        self.y = y
        self.speed = speed
        self.path = [(x, y)]

    def move(self, theta):
        x = self.speed * np.cos(theta) + self.x
        y = self.speed * np.sin(theta) + self.y
        self.x = x
        self.y = y
        self.path.append((self.x, self.y))  

    def check_move(self, theta):
        x = self.speed * np.cos(theta) + self.x
        y = self.speed * np.sin(theta) + self.y
        if ENV_MIN_X <= x and x <= ENV_MAX_X and ENV_MIN_Y <= y and y <= ENV_MAX_Y:
            pass  
        else:
            raise ValueError("Agent is out of bounds")

    def get_path(self):
        return self.path

    def get_position(self):
        return self.x, self.y

    def __getattr__(self, name):
        if name == 'x':
            return self.x
        elif name == 'y':
            return self.y
        else:
            raise AttributeError("No such attribute: {}".format(name))

def find_index(x, y):
    x_diff = x - ENV_MIN_X
    y_diff = y - ENV_MIN_Y
    # each index is incremented by 0.1
    # so the index of x_diff = 0.0 is 0
    # the index of x_diff = 0.1 is 1
    # the index of x_diff = 0.2 is 2
    # first we need to lower the x_diff and y_diff to 0.1 precision TODO
    # then we need to multiply by 10 to get the index
    x_index = int(x_diff * 10)
    y_index = int(y_diff * 10)
    return x_index, y_index

def inverse_index(x_index, y_index):
    x = x_index / 10 + ENV_MIN_X
    y = y_index / 10 + ENV_MIN_Y
    return x, y

def main():
    agent1 = Agent(0, 0, 3.05)
    agent2 = Agent(0, 1, 2.78)
    agents = [agent1, agent2]

    # construct a 4d array containing 0s
    # the first two dimensions are for the x and y coordinates of the first agent
    # the second two dimensions are for the x and y coordinates of the second agent

    environment = np.zeros(((ENV_MAX_X - ENV_MIN_X) * 10 + 1, (ENV_MAX_Y - ENV_MIN_Y) * 10 + 1,
                            (ENV_MAX_X - ENV_MIN_X) * 10 + 1, (ENV_MAX_Y - ENV_MIN_Y) * 10 + 1))

    # this is the environment only for the first agent
    # used for easy plotting purposes
    agent_environment1 = np.zeros(((ENV_MAX_X - ENV_MIN_X) * 10 + 1, (ENV_MAX_Y - ENV_MIN_Y) * 10 + 1))
    agent_environment2 = np.zeros(((ENV_MAX_X - ENV_MIN_X) * 10 + 1, (ENV_MAX_Y - ENV_MIN_Y) * 10 + 1))
    agent_environments = [agent_environment1, agent_environment2]

    # update the first environments with the positions of the agents
    agent_environments[0][find_index(agent1.x, agent1.y)] = 1
    agent_environments[1][find_index(agent2.x, agent2.y)] = 1
    environment[find_index(agent1.x, agent1.y) + find_index(agent2.x, agent2.y)] = 1

    # open two subplots
    ax1 = plt.subplot(1, 2, 1)
    ax1.set_xlim(ENV_MIN_X, ENV_MAX_X)
    ax1.set_ylim(ENV_MIN_Y, ENV_MAX_Y)
    plt.scatter(agent1.x, agent1.y, c='r')
    ax2 = plt.subplot(1, 2, 2)
    ax2.set_xlim(ENV_MIN_X, ENV_MAX_X)
    ax2.set_ylim(ENV_MIN_Y, ENV_MAX_Y)
    plt.scatter(agent2.x, agent2.y, c='r')

    # onclick event
    def onclick(event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
            (event.button, event.x, event.y, event.xdata, event.ydata))

    # connect the onclick event to the figure
    cid = plt.gcf().canvas.mpl_connect('button_press_event', onclick)

    plt.show(block=False)
    time.sleep(1)

    # start backwards rrt here, and update the plot in every 100th iteration
    while True:
        # get a random angle between 0 and 2pi
        theta = np.random.uniform(0, 2 * np.pi)
        flag = False
        for agent in agents:
            try:
                agent.check_move(theta)
            except ValueError:
                flag = True
                break
        if flag:
            continue
        agents[0].move(theta)
        print("Agent 1: ", agents[0].x, agents[0].y)
        agents[1].move(theta)
        print("Agent 2: ", agents[1].x, agents[1].y)
        # update the environment
        agent_environments[0][find_index(agents[0].x, agents[0].y)] = 1
        agent_environments[1][find_index(agents[1].x, agents[1].y)] = 1
        environment[find_index(agents[0].x, agents[0].y) + find_index(agents[1].x, agents[1].y)] = 1

        # update the plot
        ax1.scatter(agents[0].x, agents[0].y, c='b')
        ax2.scatter(agents[1].x, agents[1].y, c='b')


        plt.draw()
        plt.pause(0.0001)
        plt.show(block=False)
        print("---------------------------------")

        









if __name__ == '__main__':
    main()













    """
    # plot agent_environment1 as each 1 represents a point
    x_list_agent1 = []
    y_list_agent1 = []
    x_list_agent2 = []
    y_list_agent2 = []
    for i in range(len(agent_environment1)):
        for j in range(len(agent_environment1[i])):
            if agent_environment1[i][j] == 1:
                x, y = inverse_index(i, j)
                x_list_agent1.append(x)
                y_list_agent1.append(y)
    for i in range(len(agent_environment2)):
        for j in range(len(agent_environment2[i])):
            if agent_environment2[i][j] == 1:
                x, y = inverse_index(i, j)
                x_list_agent2.append(x)
                y_list_agent2.append(y)


    plt.scatter(x_list_agent1, y_list_agent1, c='r')
    plt.xlim(ENV_MIN_X, ENV_MAX_X)
    plt.ylim(ENV_MIN_Y, ENV_MAX_Y)
    plt.show()
    """