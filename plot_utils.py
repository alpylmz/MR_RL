import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import animation
import numpy as np
import colorsys

def plot_xy(
    xys,
    legends=[""],
    fig_title=[""]
    ):
    fig, ax = plt.subplots()

    for (X,Y),legend in zip(xys,legends):
        ax.plot(X,Y, label=legend)
   
    ax.legend(loc='upper left', 
                       shadow=True, fontsize='x-small')
    fig.suptitle(fig_title[0])
    plt.show()

def plot_bounded_curves(curves, bounds, legends=[""], fig_title=[""]):
    fig, ax = plt.subplots()
    for (t, lb, ub) in bounds:
        ax.fill_between(t, lb, ub)

    colors = cm.Set1(np.linspace(0, 1, len(curves)))
    for (X,Y),legend,c in zip(curves,legends,colors):
        ax.plot(X,Y, color=c, label=legend)


    ax.legend(loc='upper left', 
                shadow=True, fontsize='x-small')
    fig.suptitle(fig_title[0])
    plt.show()   


def plot_traj(xts, legends=[""],fig_title=[""]):
    fig, ax = plt.subplots()
    for (times,xs),legend in zip(xts,legends):
        ax.plot(times,xs,label=legend)
   
    ax.legend(loc='upper left', shadow=True, fontsize='x-small')
    fig.suptitle(fig_title[0])
    plt.show()


    
def plot_vel(vxys, legends=[""],fig_title=[""]):
    fig, ax = plt.subplots()
    figy, ay = plt.subplots()
    handles =[]
    N = int(3*len(vxys))
    HSV_tuples = [(x*1.0/N, 0.5, 0.5) for x in range(N)]
    colors =  list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))
    if N==3:
        colors =['k','b','r']
    for index,((v_desired,v_error,v_stdv,vx,vy),legend) in enumerate(zip(vxys,legends)):
        time = np.arange(len(vx))
        ax.fill_between(time,   v_desired[:,0] + v_error[:,0] - 2*v_stdv[:,0], 
                                v_desired[:,0] + v_error[:,0] + 2*v_stdv[:,0])
        ax.fill_between(time,   v_desired[:,0] + v_error[:,0] - v_stdv[:,0],   
                                v_desired[:,0] + v_error[:,0] + v_stdv[:,0])
        
        ax.plot(time, vx,                            colors[3*index+0]) #data (real from sim)
        ax.plot(time, v_desired[:,0],                colors[3*index+1]) #desired (ideal=> no noise)
        ax.plot(time, v_desired[:,0] + v_error[:,0], colors[3*index+2]) #learned (predicted from )
        
        ay.fill_between(time,   v_desired[:,1] + v_error[:,1] - 2*v_stdv[:,1], 
                                v_desired[:,1] + v_error[:,1] + 2*v_stdv[:,1])
        ay.fill_between(time,   v_desired[:,1] + v_error[:,1] - v_stdv[:,1],   
                                v_desired[:,1] + v_error[:,1] + v_stdv[:,1])
        
        ay.plot(time, vy,                            colors[3*index+0]) #data (real from sim)
        ay.plot(time, v_desired[:,1],                colors[3*index+1]) #desired (ideal=> no noise)
        ay.plot(time, v_desired[:,1] + v_error[:,1], colors[3*index+2]) #learned (predicted from )
        
        #proxy artists for figures
        h1 = mpatches.Patch(color= colors[3*index+0], label='Data_'+legend)
        h2 = mpatches.Patch(color= colors[3*index+1], label='Desired_'+legend)
        h3 = mpatches.Patch(color= colors[3*index+2], label='Learned_'+legend)
        handles.append(h1)
        handles.append(h2)
        handles.append(h3)
    
    fig.legend(handles=handles), figy.legend(handles=handles)
    fig.suptitle(fig_title[0]+'_X') ,figy.suptitle(fig_title[0]+'_Y') 
    fig.legend(loc='upper left', 
                       shadow=True, fontsize='x-small')
    figy.legend(loc='upper left', 
                       shadow=True, fontsize='x-small')
    plt.show()


def save_frames_as_gif(frames, path='./', filename='gym_animation.gif'):

    #Mess with this to change frame size
    plt.figure(figsize=(frames[0].shape[1] / 72.0, frames[0].shape[0] / 72.0), dpi=72)

    patch = plt.imshow(frames[0])
    plt.axis('off')

    def animate(i):
        patch.set_data(frames[i])

    anim = animation.FuncAnimation(plt.gcf(), animate, frames = len(frames), interval=50)
    anim.save(path + filename, writer='imagemagick', fps=60)