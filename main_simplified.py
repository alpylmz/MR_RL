import numpy as np
import Learning_module as GP # type: ignore
from utils import run_sim, find_alpha_corrected
from plot_utils import plot_xy, plot_traj, plot_bounded_curves
from scipy.ndimage import uniform_filter1d

# from MR_experiment import run_exp

dt = 0.03

#frequency of the magnetic field in Hz and the nominal value of a0
MAGNETIC_FIELD_FREQ = 4
a0_def = 1.5
SIMULATION_FREQ_IN_HZ = 30

# since it is assumed that the robots are identical, we only use one robot for these actions here!
def execute_idle_action(gp_sim: GP.LearningModule, noise_var: float = 0.0):

    step_count = 3 * SIMULATION_FREQ_IN_HZ
    actions_idle = np.zeros((step_count, 2))

    #first we will do absolutely nothing to try and calculate the drift term
    px_idle,py_idle,alpha_idle,time_idle,freq_idle = run_sim(actions_idle,
                                                                init_pos = [np.array([0,0])],
                                                                noise_var = noise_vars[i],
                                                                a0=a0_def,is_mismatched=True)


    gp_sim.estimateDisturbance(px_idle, py_idle, time_idle)

def execute_learn_action(gp_sim: GP.LearningModule, noise_var: float = 0.0, plot: bool = True):
    
    step_count = 10 * SIMULATION_FREQ_IN_HZ
    cycles = 3

    steps = (int)(step_count / cycles)

    actions_circle = np.zeros((steps, 3))
    actions_circle[:,0] = MAGNETIC_FIELD_FREQ
    actions_circle[:,1] = np.linspace(-np.pi, np.pi, steps)

    actions_learn = np.vstack([actions_circle] * cycles)
    actions_learn[:,2] = np.arange(0, dt*step_count, dt)

    t = np.linspace(0, step_count, step_count)

    # TODO: why?
    #actions_learn[:,0] = (np.cos(t / 5) + 1)/2 * 4.9 + 0.1

    # THIS IS WHAT THE SIMULATION ACTUALLY GIVES US -- model mismatch && noise
    px_sim,py_sim,alpha_sim,time_sim,freq_sim = run_sim(actions_learn,
                                                         init_pos = np.array([0,0]),
                                                         noise_var = noise_vars[i],
                                                         a0 = a0_def,
                                                         is_mismatched = True)


    # learn noise and a0 -- note px_desired and py_desired need to be at the same time
    a0_sim = gp_sim.learn(px_sim, py_sim, alpha_sim, time_sim, actions)
    print("Estimated a0 value is " + str(a0_sim))
    gp_sim.visualize()

    
    # THIS CALCULATES THE DESIRED TRAJECTORY FROM OUR a0 ESTIMATE
    px_desired,py_desired,alpha_desired,time_desired,freq_desired = run_sim(actions_learn,
                                                                            init_pos = np.array([0,0]),
                                                                            noise_var = 0.0,
                                                                            a0 = a0_sim)

    if plot:
        # plot the desired vs achieved velocities
        xys  = [(px_desired,py_desired),
                (px_sim,py_sim),
            ]
        legends =["Desired Trajectory","Simulated Trajectory (no learning)"
                ]
        fig_title   = ["Learning Dataset"]
        plot_xy(xys,legends =legends,fig_title =fig_title) 

    return a0_sim


#actions_learn[:,0] = (np.cos(t / 5) + 1)/2 * 4.9 + 0.1

#generate actions for testing (1/30 hz for 30 seconds)
time_steps = 1000

actions = np.zeros( (time_steps, 3) )

actions[0:200,1]   = np.linspace(0, np.pi/2, 200)
actions[200:400,1] = np.linspace(np.pi/2, -np.pi/2, 200)
actions[400:600,1] = np.linspace(-np.pi/2, 0, 200)
actions[600:800,1] = np.linspace(0, np.pi/8, 200)
actions[800::,1]  = np.linspace(np.pi/8, -np.pi, 200)

actions[:,0] = MAGNETIC_FIELD_FREQ # np.linspace(3, 4, time_steps)
actions[:,2] = np.arange(0, dt*time_steps, dt)

#actions = np.array([[1, 0.3*np.pi*((t/time_steps)-1)*(-1)**(t//300)] 
#                        for t in range(1,time_steps)]) # [T,action_dim]


noise_vars= [0.0]
for i in range(len(noise_vars)):


    gp_sim = GP.LearningModule()

    execute_idle_action(gp_sim, noise_vars[i])
    
    a0_sim = execute_learn_action(gp_sim, noise_vars[i])
    ###### END OF LEARNING, NOW WE DO TESTING ######


    # Desired Trajectory: no noise, no learning -- this is the desired trajectory
    px_desired,py_desired,alpha_desired,time_desired,freq_desired = run_sim(actions,
                                                             init_pos = np.array([0,0]),
                                                             noise_var = 0.0,
                                                             a0=a0_sim) #assume we used a0_sim to generate the control actions

    # Baseline: actual noise and parameters, no learning -- this is the achieved trajectory
    px_baseline,py_baseline,alpha_baseline, time_baseline,freq_baseline = run_sim(actions,
                                                         init_pos = np.array([0,0]),
                                                         noise_var = noise_vars[i],
                                                         a0=a0_def,is_mismatched=True)

    #generate our desired, predicted, and error bars for velocity for the test
    vd = np.zeros( (len(actions), 2) )
    v_pred = np.zeros( (len(actions), 2) )
    v_stdv  = np.zeros( (len(actions), 2) )
    actions_corrected = np.zeros(actions.shape)

    for ii in range(len(actions_corrected)):
        vd[ii,:] = a0_sim*MAGNETIC_FIELD_FREQ*np.array( [np.cos(actions[ii,1]), np.sin(actions[ii,1])] ).reshape(1,-1)
        #actions_corrected[ii,0] = actions[ii,0] #don't correct the rolling frequency
        A, muX, muY, sigX, sigY = find_alpha_corrected(vd[ii],gp_sim)
        
        actions_corrected[ii,0] = actions[ii,0]
        actions_corrected[ii,1] = A
        
        #our predicted velocity is model + error
        v_pred[ii,0] = a0_sim*MAGNETIC_FIELD_FREQ*np.cos(actions_corrected[ii,1]) + muX
        v_pred[ii,1] = a0_sim*MAGNETIC_FIELD_FREQ*np.sin(actions_corrected[ii,1]) + muY
        v_stdv[ii,0] = sigX
        v_stdv[ii,1] = sigY
        
    
     # sim: noise, learning
    px_learn,py_learn,alpha_learn, time_learn,freq_learn = run_sim(actions_corrected,
                                                         init_pos = np.array([0,0]),
                                                         noise_var = noise_vars[i],
                                                         a0=a0_def,is_mismatched=True) #simulate using the true value of a0
    
    
    #### Plot Resulting Trajectories
    xys = [(px_desired,py_desired),
           (px_baseline,py_baseline),
           (px_learn,py_learn)]
    legends= ["desired",
                "baseline",
                "learning"]
    plot_xy(xys,legends =legends,fig_title =["Trajectories"]) 

    
    alphas = [(time_baseline,actions[:,1]),
              (time_learn,actions_corrected[:,1]) ]
    
    plot_traj(alphas,legends =['alpha',
                                      'alpha_corrected'],fig_title =["alphas"])
    
    
    ### plot x and y velocity bounds
    N=14
    px_learn = uniform_filter1d(px_learn, N, mode="nearest")
    px_learn = uniform_filter1d(px_learn, N, mode="nearest")
    vx_learn = np.gradient(px_learn, time_learn)
    vy_learn = np.gradient(py_learn, time_learn)
    vx_learn = uniform_filter1d(vx_learn, (int)(N/2), mode="nearest")
    vy_learn = uniform_filter1d(vy_learn, (int)(N/2), mode="nearest")

    vx_baseline = np.gradient(px_baseline, time_baseline)
    vy_baseline = np.gradient(py_baseline, time_baseline)
    vx_baseline = uniform_filter1d(vx_baseline, (int)(N/2), mode="nearest")
    vy_baseline = uniform_filter1d(vy_baseline, (int)(N/2), mode="nearest")


    vx_curve = [(time_learn,      vx_learn),
                (time_baseline[N:-N], vx_baseline[N:-N]),
                (time_desired,  a0_def*MAGNETIC_FIELD_FREQ*np.cos(alpha_desired))]
    vx_bounds   = []#[(time_learn, v_pred[:,0]+2*v_stdv[:,0], v_pred[:,0]-2*v_stdv[:,0]), 
                #(time_learn, v_pred[:,0]+v_stdv[:,0], v_pred[:,0]-v_stdv[:,0])]
    plot_bounded_curves(vx_curve,vx_bounds,legends=['learning', 'uncorrected', 'desired'], fig_title=["Vx Profile"])

    vy_curve = [(time_learn,    vy_learn),
                (time_baseline[N:-N], vy_baseline[N:-N]),
                (time_desired,  a0_def*MAGNETIC_FIELD_FREQ*np.sin(alpha_desired))]
    vy_bounds   = []# [(time_learn, v_pred[:,1]+2*v_stdv[:,1], v_pred[:,1]-2*v_stdv[:,1]), 
                    #(time_learn, v_pred[:,1]+v_stdv[:,1], v_pred[:,1]-v_stdv[:,1])]
    plot_bounded_curves(vy_curve,vy_bounds,legends=['learning', 'uncorrected', 'desired'], fig_title=["Vy Profile"])



    ###plot the desired vs actual velocity with the GP bounds -- see if we learned the error or not
    vx_desired = a0_sim * MAGNETIC_FIELD_FREQ * np.cos(alpha_desired)
    vy_desired = a0_sim * MAGNETIC_FIELD_FREQ * np.sin(alpha_desired)

    vel_error = np.zeros( (len(time_desired), 2) )
    vel_sigma = np.zeros( (len(time_desired), 2) )
    for ti in range(len(time_desired)):
        muX, muY, sigX, sigY = gp_sim.error( [vx_desired[ti], vy_desired[ti]] )
        
        vel_error[ti,:] = np.array([muX, muY]).reshape(1,-1)

        vel_sigma[ti,:] = np.array([sigX, sigY]).reshape(1,-1)



    vx_curve = [(time_baseline, vx_baseline),
                (time_desired,  vx_desired),
                (time_desired,  vx_desired + vel_error[:,0])]
    vx_bounds   = [(time_desired, vx_desired + vel_error[:,0] + 2*vel_sigma[:,0], vx_desired + vel_error[:,0] - 2*vel_sigma[:,0]), 
                   (time_desired, vx_desired + vel_error[:,0] + vel_sigma[:,0], vx_desired + vel_error[:,0] - vel_sigma[:,0])]

    plot_bounded_curves(vx_curve,vx_bounds,legends=['baseline', 'desired', 'estimate'], fig_title=["Estimating Vx Error"])


    vy_curve = [(time_baseline, vy_baseline),
                (time_desired,  vy_desired),
                (time_desired,  vy_desired + vel_error[:,1])]
    vy_bounds   = [(time_desired, vy_desired + vel_error[:,1] + 2*vel_sigma[:,1], vy_desired + vel_error[:,1] - 2*vel_sigma[:,1]), 
                   (time_desired, vy_desired + vel_error[:,1] + vel_sigma[:,1], vy_desired + vel_error[:,1] - vel_sigma[:,1])]

    plot_bounded_curves(vy_curve,vy_bounds,legends=['baseline', 'desired', 'estimate'], fig_title=["Estimating Vy Error"])


    # # time        = np.arange(1,time_steps) #start at t=0 for readability of the final graph
    # # alpha_sim   = actions[:,1]
    
    # # Case1: noise, noise correction
    # alpha_est1, vxys_1 =test_gp(gp_sim,px_sim,py_sim,
    #                      a0_def,alpha_sim,freq_sim[0],time_sim) # create a LearningModule object
    # v_desired1,v_error1,v_stdv,vx,vy = vxys_1
    # _,v_cmd1  = find_alpha_corrected(v_desired1,v_error1)# estimate correct alphas with default a0
    # actions1    = np.array([[1,alph ] for alph in alpha_est1]) # [T,action_dim]
    # px_c1,py_c1,_,_,_ = run_sim(actions1,
    #                             init_pos = np.array([0,0]),
    #                             noise_var = noise_vars[i],
    #                             a0=a0_def)
    
    # # Case3: noise, learned a0, learned error 
    # alpha_est2, vxys_2 = test_gp(gp_sim,px_sim,py_sim,
    #                      a0_sim,alpha_sim,freq_sim[0],time_sim) # create a LearningModule object
    # v_desired2,v_error2,v_stdv,vx,vy = vxys_2
    # _,v_cmd2 = find_alpha_corrected(v_desired2, v_error2)# estimate correct alphas with learned a0
    
    # actions2 = np.array([[1,alph] for alph in alpha_est2]) # [T,action_dim]
    # px_c2,py_c2,_,_,_ = run_sim(actions2,
    #                             init_pos = np.array([0,0]),
    #                             noise_var = noise_vars[i],
    #                             a0=a0_sim)
    
    # # Plot Control results 
    # xys  = [(px_base,py_base),
    #        (px_sim,py_sim),
    #        (px_c1,py_c1),
    #        (px_c2,py_c2)
    #        ]
    # v_sim = np.array([[i,j] for i,j in zip(vx,vy)])
    # alpha_sim_recovered,_= find_alpha_corrected(v_desired1, np.zeros(v_error1.shape))
    
    # alphas = [(time_sim,alpha_sim),
    #           (time_sim,alpha_sim_recovered),
    #        # (time_sim,alpha_est1),
    #        (time_sim,alpha_est2)
    #        ]
    
    # vxs = [(time_sim,v_sim[:,0]),
    #        (time_sim,v_cmd1[:,0]),
    #        (time_sim,v_cmd2[:,0])
    #        ]
    # vys = [(time_sim,v_sim[:,1]),
    #        (time_sim,v_cmd1[:,1]),
    #        (time_sim,v_cmd2[:,1])
    #        ]
    # v_errs = [(time_sim,v_error1[:,0]),
    #        (time_sim,v_error1[:,1]),
    #        (time_sim,v_error2[:,0]),
    #        (time_sim,v_error2[:,1])
    #        ]
    # vxys = [vxys_1,
    #         vxys_2
    #        ]
    
    # legends =["base (no noise)",
    #           "sim with a0",
    #           "w/noise correction",
    #           "w/noise correction + a0_learned"
    #           ]
    
    # fig_title   = ["TEST"]
    # plot_xy(xys,legends =legends,fig_title =["Trajectories"]) 
    # plot_traj(alphas,legends =['alpha_sim','alpha_sim_recovered',
    #                                   # 'alpha_cmd1',
    #                                   'alpha_cmd2'],fig_title =["alphas"])
    # # plot_traj(vxs, legends = ['v_sim', 'v_cmd1','v_cmd2'],fig_title =["x-velocities"])
    # # plot_traj(vys, legends = ['v_sim', 'v_cmd1','v_cmd2'],fig_title =["y-velocities"])
    # # plot_traj(v_errs, legends = ['v_errorX1', 'v_errorY1','v_errorX2','v_errorY2'],fig_title =["v-errors"])
    # # plot_vel([vxys_1],legends =['def a0'],fig_title =fig_title) 
    # plot_vel([vxys_2],legends =['learned a0'],fig_title =fig_title) 
    
    break
# v_desired,v_error,v_stdv,vx,vy = vxys_1