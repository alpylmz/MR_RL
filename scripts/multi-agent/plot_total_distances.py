import pickle
from const import NUMBER_OF_AGENTS

with open('total_distances.pickle', 'rb') as f:
    alphas = pickle.load(f)

#alphas = alphas[:5000]
   


import matplotlib.pyplot as plt

plt.figure()

# plot alphas as a line graph
plt.plot(alphas, linewidth = 0.4)

# add labels
plt.xlabel('Step number')
plt.ylabel('Total distance')

plt.savefig("total_distances.png", dpi = 1200)
