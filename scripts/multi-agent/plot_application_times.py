import pickle
from const import NUMBER_OF_AGENTS

with open('application_times.pickle', 'rb') as f:
    alphas = pickle.load(f)

alphas = alphas[:2000]

# separate the alphas, first element to the first list, second element to the second list, third element to the third list, andd 4th element to the fourth list, 5th element to the fifth list, and this continues with the first list again
alphas_lists = [[] for i in range(NUMBER_OF_AGENTS)]
for i in range(len(alphas)//NUMBER_OF_AGENTS):
    for j in range(NUMBER_OF_AGENTS):
        alphas_lists[j].append(alphas[i*NUMBER_OF_AGENTS + j])
    


import matplotlib.pyplot as plt

plt.figure()

# plot alphas as a line graph
plt.plot(alphas_lists[0], linewidth = 1.0)
plt.plot(alphas_lists[1], linewidth = 1.0)
plt.plot(alphas_lists[2], linewidth = 1.0)
plt.plot(alphas_lists[3], linewidth = 1.0)

# scale x on log scale
#plt.xscale('log')
# add legend
plt.legend(['1 Hz Frequency', '5 Hz Frequency', '10 Hz Frequency', '20 Hz Frequency'])

# add labels
plt.xlabel('Step number')
plt.ylabel('Application Time (s)')

plt.savefig("application_times.png", dpi = 1200)
