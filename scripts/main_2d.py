import numpy as np
from scipy.ndimage import uniform_filter1d

from utils import run_sim, find_alpha_corrected
from plot_utils import plot_xy, plot_traj, plot_bounded_curves
import Learning_module_2d as GP

SIMULATION_FREQ_IN_HZ = 30

#frequency of the magnetic field in Hz and the nominal value of a0
MAGNETIC_FIELD_FREQ = 4
# TODO: How this value is decided?
A0_DEF = 1.5

def execute_idle_action(gp_sim: GP.LearningModule, noise_var = 0.0):
    """
    Execute the idle action to learn about the noise
    Input:
        gp_sim: the GP module    
    Output:
        Returns nothing, but updates the GP object
    """
    step_count = 3 * SIMULATION_FREQ_IN_HZ # do nothing for 3 seconds
    actions_idle = np.zeros((step_count, 2))

    #first we will do absolutely nothing to try and calculate the drift term
    px_idle, py_idle, alpha_idle, time_idle, freq_idle = run_sim(
                                                            actions_idle,
                                                            init_pos = [np.array([0, 0])],
                                                            noise_var = noise_var,
                                                            a0 = A0_DEF,
                                                            is_mismatched = False
                                                            )

    gp_sim.estimateDisturbance(px_idle, py_idle, time_idle)


def execute_learn_action(gp_sim: GP.LearningModule, noise_var = 0.0, plot = True):
    """
    Execute the learn action to learn about the noise
    Input:
        gp_sim: the GP module
        plot: whether to plot the results
    Output:
        a0_sim: calculated a0 from the sim
    """
    #note: timestep is 1/30 seconds, the rate we get data at in the experiment
    step_count = 10 * SIMULATION_FREQ_IN_HZ # train for 10s at 30 hz
    cycles = 3 # train my moving in 3 circles

    steps = (int)(step_count / cycles)

    #generate actions to move in a circle at a constant frequency
    actions_circle = np.zeros((steps, 2)) 
    actions_circle[:,0] = MAGNETIC_FIELD_FREQ
    actions_circle[:,1] = np.linspace(-np.pi, np.pi, steps)

    #stack the circle actions to get our learning set
    actions_learn = np.vstack([actions_circle] * cycles)

    t = np.linspace(0, step_count, step_count)

    # TODO: why?
    #actions_learn[:,0] = (np.cos(t / 5) + 1)/2 * 4.9 + 0.1
    
    
    # THIS IS WHAT THE SIMULATION ACTUALLY GIVES US -- model mismatch && noise
    x_sim, y_sim, alpha_sim, time_sim, freq_sim = run_sim(
                                                        actions_learn,
                                                        init_pos = [np.array([0, 0])],
                                                        noise_var = noise_var,
                                                        a0 = A0_DEF,
                                                        is_mismatched = False
                                                        )


    # learn noise and a0 -- note px_desired and py_desired need to be at the same time
    a0_sim = gp_sim.learn(x_sim, y_sim, alpha_sim, freq_sim, time_sim)
    print("Estimated a0 value is " + str(a0_sim))
    if plot:
        gp_sim.visualize()

    # THIS CALCULATES THE DESIRED TRAJECTORY FROM OUR a0 ESTIMATE
    x_desired, y_desired, alpha_desired, time_desired, freq_desired = run_sim(
                                                                    actions_learn,
                                                                    init_pos = [np.array([0,0])],
                                                                    noise_var = 0.0,
                                                                    a0 = a0_sim,
                                                                    is_mismatched = False
                                                                    )
    if plot:
        # plot the desired vs achieved velocities
        xys  = [
                (x_desired, y_desired),
                (x_sim, y_sim),
            ]
        legends = ["Desired Trajectory","Simulated Trajectory (no learning)"]
        fig_title = ["Learning Dataset"]
        plot_xy(xys, legends = legends, fig_title = fig_title) 

    return a0_sim


def execute_test_action():
    """
    This calculates the desired trajectory, with no noise and no learning, 
    and the baseline trajectory, with the actual noise and parameters but no learning
    Output:
        actions_testing: the used alpha and frequency values 
        x_desired: the desired x trajectory
        y_desired: the desired y trajectory
        alpha_desired: the desired alpha trajectory
        time_desired: the desired time trajectory
        freq_desired: the desired frequency trajectory
        x_baseline: the baseline x trajectory
        y_baseline: the baseline y trajectory
        alpha_baseline: the baseline alpha trajectory
        time_baseline: the baseline time trajectory
        freq_baseline: the baseline frequency trajectory
    """

    #generate actions for testing (1/30 hz for 30 seconds)
    step_count = 33 * SIMULATION_FREQ_IN_HZ

    actions_testing = np.zeros((step_count, 2))

    actions_testing[0:200,1]   = np.linspace(0,         np.pi/2,    200)
    actions_testing[200:400,1] = np.linspace(np.pi/2,   -np.pi/2,   200)
    actions_testing[400:600,1] = np.linspace(-np.pi/2,  0,          200)
    # TODO: why?
    actions_testing[600:800,1] = np.linspace(0,         np.pi/8,    200)
    actions_testing[800::,1]   = np.linspace(np.pi/8,   -np.pi,     190)

    actions_testing[:,0] = MAGNETIC_FIELD_FREQ # np.linspace(3, 4, time_steps)

    # Desired Trajectory: no noise, no learning -- this is the desired trajectory
    x_desired, y_desired, alpha_desired, time_desired, freq_desired = run_sim(
                                                            actions_testing,
                                                            init_pos = np.array([0,0]),
                                                            noise_var = 0.0,
                                                            a0 = a0_sim
                                                            ) #assume we used a0_sim to generate the control actions

    # Baseline: actual noise and parameters, no learning -- this is the achieved trajectory
    x_baseline, y_baseline, alpha_baseline, time_baseline, freq_baseline = run_sim(
                                                            actions_testing,
                                                            init_pos = np.array([0,0]),
                                                            noise_var = noise_vars[i],
                                                            a0 = A0_DEF,
                                                            is_mismatched = True
                                                            )

    return actions_testing, x_desired, y_desired, alpha_desired, time_desired, freq_desired, x_baseline, y_baseline, alpha_baseline, time_baseline, freq_baseline


if __name__ == "__main__":

    noise_vars= [0.0]
    for i in range(len(noise_vars)):

        gp_sim = GP.LearningModule()
    
        execute_idle_action(gp_sim)

        a0_sim = execute_learn_action(gp_sim, plot = True)
        
        ###### END OF LEARNING, NOW WE DO TESTING ######

        actions_testing, x_desired, y_desired, alpha_desired, time_desired, freq_desired, x_baseline, y_baseline, alpha_baseline, time_baseline, freq_baseline = execute_test_action()

        #generate our desired, predicted, and error bars for velocity for the test
        vd = np.zeros((len(actions_testing), 2))
        v_pred = np.zeros((len(actions_testing), 2))
        v_stdv = np.zeros((len(actions_testing), 2))
        # TODO: What we are doing with this variable?
        actions_corrected = np.zeros(actions_testing.shape)

        for ii in range(len(actions_corrected)):
            vd[ii,:] = a0_sim * MAGNETIC_FIELD_FREQ * np.array([np.cos(actions_testing[ii, 1]), np.sin(actions_testing[ii, 1])] ).reshape(1, -1)
            #actions_corrected[ii,0] = actions[ii,0] #don't correct the rolling frequency
            A, muX, muY, sigX, sigY = find_alpha_corrected(vd[ii], gp_sim)
            
            actions_corrected[ii,0] = A[1]
            actions_corrected[ii,1] = A[0]
            
            #our predicted velocity is model + error
            v_pred[ii,0] = a0_sim*MAGNETIC_FIELD_FREQ*np.cos(actions_corrected[ii,1]) + muX
            v_pred[ii,1] = a0_sim*MAGNETIC_FIELD_FREQ*np.sin(actions_corrected[ii,1]) + muY
            v_stdv[ii,0] = sigX
            v_stdv[ii,1] = sigY
            
        
        # sim: noise, learning
        # TODO: Why a0 = A0_DEF?
        px_learn, py_learn, alpha_learn, time_learn, freq_learn = run_sim(
                                                                actions_corrected,
                                                                init_pos = np.array([0,0]),
                                                                noise_var = 0.0,
                                                                a0 = A0_DEF,
                                                                is_mismatched = True
                                                                ) #simulate using the true value of a0
        
        



        #### Plot Resulting Trajectories
        xys = [(x_desired, y_desired),
            (x_baseline, y_baseline),
            (px_learn, py_learn)]
        legends= [
                "desired",
                "baseline",
                "learning"]
        plot_xy(xys,legends =legends,fig_title =["Trajectories"]) 

        
        alphas = [(time_baseline, actions_testing[:,1]),
                (time_learn, actions_corrected[:,1])]
        
        plot_traj(
            alphas, 
            legends = [
                'alpha',
                'alpha_corrected'
            ],
            fig_title = ["alphas"], 
            )
        
        
        ### plot x and y velocity bounds
        N = 14
        px_learn = uniform_filter1d(px_learn, N, mode="nearest")
        px_learn = uniform_filter1d(px_learn, N, mode="nearest")
        vx_learn = np.gradient(px_learn, time_learn)
        vy_learn = np.gradient(py_learn, time_learn)
        vx_learn = uniform_filter1d(vx_learn, (int)(N/2), mode="nearest")
        vy_learn = uniform_filter1d(vy_learn, (int)(N/2), mode="nearest")

        vx_baseline = np.gradient(x_baseline, time_baseline)
        vy_baseline = np.gradient(y_baseline, time_baseline)
        vx_baseline = uniform_filter1d(vx_baseline, (int)(N/2), mode="nearest")
        vy_baseline = uniform_filter1d(vy_baseline, (int)(N/2), mode="nearest")


        vx_curve = [(time_learn, vx_learn),
                    (time_baseline[N:-N], vx_baseline[N:-N]),
                    (time_desired, A0_DEF * MAGNETIC_FIELD_FREQ * np.cos(alpha_desired))]
        vx_bounds = []#[(time_learn, v_pred[:,0]+2*v_stdv[:,0], v_pred[:,0]-2*v_stdv[:,0]), 
                    #(time_learn, v_pred[:,0]+v_stdv[:,0], v_pred[:,0]-v_stdv[:,0])]
        plot_bounded_curves(
                        vx_curve, 
                        vx_bounds, 
                        legends=[
                            'learning', 
                            'uncorrected', 
                            'desired'
                        ], 
                        fig_title=["Vx Profile"],
                        )

        vy_curve = [(time_learn, vy_learn),
                    (time_baseline[N:-N], vy_baseline[N:-N]),
                    (time_desired, A0_DEF * MAGNETIC_FIELD_FREQ * np.sin(alpha_desired))]
        vy_bounds = []# [(time_learn, v_pred[:,1]+2*v_stdv[:,1], v_pred[:,1]-2*v_stdv[:,1]), 
                        #(time_learn, v_pred[:,1]+v_stdv[:,1], v_pred[:,1]-v_stdv[:,1])]
        plot_bounded_curves(
                    vy_curve, 
                    vy_bounds, 
                    legends = [
                        'learning', 
                        'uncorrected', 
                        'desired'
                    ], 
                    fig_title=["Vy Profile"],
                    )



        ###plot the desired vs actual velocity with the GP bounds -- see if we learned the error or not
        vx_desired = a0_sim * MAGNETIC_FIELD_FREQ * np.cos(alpha_desired)
        vy_desired = a0_sim * MAGNETIC_FIELD_FREQ * np.sin(alpha_desired)

        vel_error = np.zeros((len(time_desired), 2))
        vel_sigma = np.zeros((len(time_desired), 2))
        for ti in range(len(time_desired)):
            muX, muY, sigX, sigY = gp_sim.error([vx_desired[ti], vy_desired[ti]])
            
            vel_error[ti,:] = np.array([muX, muY]).reshape(1,-1)

            vel_sigma[ti,:] = np.array([sigX, sigY]).reshape(1,-1)



        vx_curve = [(time_baseline, vx_baseline),
                    (time_desired,  vx_desired),
                    (time_desired,  vx_desired + vel_error[:,0])]
        vx_bounds = [
            (
                time_desired, 
                vx_desired + vel_error[:,0] + 2 * vel_sigma[:,0], 
                vx_desired + vel_error[:,0] - 2 * vel_sigma[:,0]
            ), 
            (
                time_desired, 
                vx_desired + vel_error[:,0] + vel_sigma[:,0], 
                vx_desired + vel_error[:,0] - vel_sigma[:,0]
            )]

        plot_bounded_curves(
                    vx_curve, 
                    vx_bounds, 
                    legends=[
                        'baseline', 
                        'desired', 
                        'estimate'
                    ], 
                    fig_title = [
                        "Estimating Vx Error"
                    ])


        vy_curve = [(time_baseline, vy_baseline),
                    (time_desired,  vy_desired),
                    (time_desired,  vy_desired + vel_error[:,1])]
        vy_bounds = [
            (
                time_desired, 
                vy_desired + vel_error[:,1] + 2*vel_sigma[:,1], 
                vy_desired + vel_error[:,1] - 2*vel_sigma[:,1]
            ), 
            (
                time_desired, 
                vy_desired + vel_error[:,1] + vel_sigma[:,1], 
                vy_desired + vel_error[:,1] - vel_sigma[:,1]
            )]

        plot_bounded_curves(
            vy_curve,
            vy_bounds,
            legends = [
                'baseline', 
                'desired', 
                'estimate'
            ], 
            fig_title = [
                "Estimating Vy Error"
            ])

        # TODO: why?
        break