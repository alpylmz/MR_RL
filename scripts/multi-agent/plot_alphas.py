import pickle
from const import NUMBER_OF_AGENTS

with open('plotted_alphas.pickle', 'rb') as f:
    alphas = pickle.load(f)

import matplotlib.pyplot as plt

plt.figure()

# plot alphas as a line graph
plt.plot(alphas[:300], linewidth = 0.2)

plt.savefig("alphas.png", dpi = 1200)
