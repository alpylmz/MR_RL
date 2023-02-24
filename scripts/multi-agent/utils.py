import numpy as np
from const import *
import random

def generate_random_frequencies2N():

    frequencies = []
    # generate a random 2N*2N frequency matrix
    for i in range(2 * NUMBER_OF_AGENTS):
        lst = []
        for j in range(2 * NUMBER_OF_AGENTS):
            lst.append(random.randint(1, 20))
        # make the unwanted frequencies zero
        # for example, if NUMBER_OF_AGENTS = 2, then the frequencies should be like this:
        # [[1, 2, 0, 0],
        #  [0, 0, 1, 3]]
        # for NUMBER_OF_AGENTS = 3, the frequencies should be like this:
        # [[1, 2, 0, 0, 0, 0],
        #  [0, 0, 1, 3, 0, 0],
        #  [0, 0, 0, 0, 1, 4]]
        # and so on
        for j in range(2 * NUMBER_OF_AGENTS):
            if i >= NUMBER_OF_AGENTS and j >= NUMBER_OF_AGENTS:
                lst[j] = 0
            if i < NUMBER_OF_AGENTS and j < NUMBER_OF_AGENTS:
                lst[j] = 0
        frequencies.append(lst)
    return frequencies

def generate_random_frequencies():
    while True:
        frequencies = []
        # generate a random N*N frequency matrix
        for i in range(NUMBER_OF_AGENTS):
            lst = []
            for j in range(NUMBER_OF_AGENTS):
                lst.append(random.randint(1, 20))
            frequencies.append(lst)

        # calculate determinant to make sure that the frequencies are linearly independent
        det = np.linalg.det(frequencies)
        if det == 0:
            continue
        break
    return frequencies

def generate_random_positions():
    while True:
        positions = []
        for i in range(NUMBER_OF_AGENTS):
            positions.append([random.randint(20, 70), random.randint(20, 70)])

        # check the distances between the agents, make sure they are not too close
        too_close = False
        for i in range(NUMBER_OF_AGENTS):
            for j in range(i + 1, NUMBER_OF_AGENTS):
                if np.linalg.norm(np.array(positions[i]) - np.array(positions[j])) < 1:
                    too_close = True
                    break
            if too_close:
                break
        if too_close:
            continue
        return positions

