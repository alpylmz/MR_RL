import numpy as np
import matplotlib.pyplot as plt
import time

ENV_MIN_X = -1
ENV_MAX_X = 1
ENV_MIN_Y = -1
ENV_MAX_Y = 1

class Agent():
    def __init__(self, x, y, speed) -> None:
        self.x = x
        self.y = y
        self.speed = speed
        self.path = [(x, y)]

    def move(self, theta):
        self.x += (self.speed * np.cos(theta)) * 0.07
        self.y += (self.speed * np.sin(theta)) * 0.07
        self.path.append((self.x, self.y))  

    def check_move(self, theta):
        x = self.x + (np.cos(theta) * self.speed) * 0.07
        y = self.y + (np.sin(theta) * self.speed) * 0.07
        if ENV_MIN_X <= x and x <= ENV_MAX_X and ENV_MIN_Y <= y and y <= ENV_MAX_Y:
            pass  
        else:
            raise ValueError("Agent is out of bounds")
        return x, y

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


def main():
    agent1 = Agent(0, 0, 3.72)
    agent2 = Agent(0, 0.1, 1.4)
    agents = [agent1, agent2]

    # construct a 4d array containing 0s
    # the first two dimensions are for the x and y coordinates of the first agent
    # the second two dimensions are for the x and y coordinates of the second agent

    configurations = {}

    configurations[agent1.x] = {}
    configurations[agent1.x][agent1.y] = {}
    configurations[agent1.x][agent1.y][agent2.x] = {}
    configurations[agent1.x][agent1.y][agent2.x][agent2.y] = 0


    # open two subplots
    #ax1 = plt.subplot(1, 2, 1)
    #ax1.set_xlim(ENV_MIN_X, ENV_MAX_X)
    #ax1.set_ylim(ENV_MIN_Y, ENV_MAX_Y)
    #plt.scatter(agent1.x, agent1.y, c='r')
    #ax2 = plt.subplot(1, 2, 2)
    #ax2.set_xlim(ENV_MIN_X, ENV_MAX_X)
    #ax2.set_ylim(ENV_MIN_Y, ENV_MAX_Y)
    #plt.scatter(agent2.x, agent2.y, c='r')

    # onclick event
    def onclick(event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
            (event.button, event.x, event.y, event.xdata, event.ydata))

    # connect the onclick event to the figure
    cid = plt.gcf().canvas.mpl_connect('button_press_event', onclick)

    plt.show(block=False)
    time.sleep(1)

    # start backwards rrt here, and update the plot in every 100th iteration
    from tqdm import tqdm
    num_of_ones = 0

    def find_new_angle(configurations, agents):
        # choose a random angle
        # check if the new positions of the agents have been visited
        # if not, return the angle
        # if yes, choose a new angle
        # if all angles have been tried, return a random angle

        # get a random angle between 0 and 2pi
        i = 0
        while True:
            #print("starting loop")
            theta = np.random.uniform(0, 2 * np.pi)
            flag = False
            xs = []
            ys = []
            #print("Current coordinates: {}, {}".format(agents[0].x, agents[0].y))
            #print("Current coordinates: {}, {}".format(agents[1].x, agents[1].y))
            for agent in agents:
                try:
                    x, y = agent.check_move(theta)
                    xs.append(x)
                    ys.append(y)
                except ValueError:
                    flag = True
                    break
            if flag:
                # if the angle is not valid, return a random angle
                continue
            # I put this here to make sure that the angle is valid
            i += 1
            if i > 100:
                return theta
            #print("New coordinates: {}, {}".format(xs[0], ys[0]))
            #print("New coordinates: {}, {}".format(xs[1], ys[1]))
            #print("angle valid")
            # check if the new positions of the agents have been visited
            if xs[0] in configurations:
                if ys[0] in configurations[xs[0]]:
                    if xs[1] in configurations[xs[0]][ys[0]]:
                        if ys[1] in configurations[xs[0]][ys[0]][xs[1]]:
                            # if the new positions have been visited, choose a new angle
                            continue
            # if the new positions have not been visited, return the angle
            return theta


    for j in tqdm(range(100000)):
        #print("Iteration: ", j)
        # get a random angle between 0 and 2pi
        theta = find_new_angle(configurations, agents)
        agents[0].move(theta)
        #print("Agent 1: ", agents[0].x, agents[0].y)
        agents[1].move(theta)
        #print("Agent 2: ", agents[1].x, agents[1].y)
        # update the environment
        if agents[0].x not in configurations:
            configurations[agents[0].x] = {}
        if agents[0].y not in configurations[agents[0].x]:
            configurations[agents[0].x][agents[0].y] = {}
        if agents[1].x not in configurations[agents[0].x][agents[0].y]:
            configurations[agents[0].x][agents[0].y][agents[1].x] = {}
        if agents[1].y not in configurations[agents[0].x][agents[0].y][agents[1].x]:
            configurations[agents[0].x][agents[0].y][agents[1].x][agents[1].y] = 0

            #print("New configuration added!")
            #print(f"{agents[0].x} {agents[0].y} {agents[1].x} {agents[1].y}")
            num_of_ones += 1


    print("dumping environment")
    print("Number of ones: ", num_of_ones)
    import pickle
    with open("configurations.pickle", "wb") as f:
        pickle.dump(configurations, f)
    # get the size of the environment in MB
    
    print("dumped environment")
    exit(1)
    









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