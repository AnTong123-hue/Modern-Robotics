import mobile_manipulation as mm
import modern_robotics as mr
import numpy as np
import copy
import matplotlib.pyplot as plt
import argparse
import os
import csv

def save_result(csvdata, subdir, filename, opt=1):
    with open(f"./results/{subdir}/{filename}.csv", 'w', newline='') as save_file:
        writer = csv.writer(save_file)
        if (opt == 1):
            writer.writerows(csvdata)
        else:
            writer.writerow(csvdata)

def main(data='best', controller='FPI'):
    # Create error log file
    result_dir = "./results"
    sub_dir    = f"./{result_dir}/{data}"
    log_file = f"{data}_cfg_{controller}_control.log"
    log_fig  = f"{data}_cfg_{controller}_control.png"
    
    if not os.path.exists(result_dir):
        os.makedirs(result_dir)

    if not os.path.exists(sub_dir):
        os.makedirs(sub_dir)

    # Trajectory generation
    T_sc_initial    = np.array([[1.0, 0.0, 0.0, 1.0  ], 
                                [0.0, 1.0, 0.0, 0.0  ], 
                                [0.0, 0.0, 1.0, 0.025], 
                                [0.0, 0.0, 0.0, 1.0  ]])
    
    T_sc_final      = np.array([[ 0.0, 1.0, 0.0,  0.0  ], 
                                [-1.0, 0.0, 0.0, -1.0  ], 
                                [ 0.0, 0.0, 1.0,  0.025], 
                                [ 0.0, 0.0, 0.0,  1.0  ]])
    
    T_se_initial    = np.array([[0.0, 0.0, 1.0, 0.0  ], 
                                [0.0, 1.0, 0.0, 0.0  ], 
                                [-1.0, 0.0, 0.0, 0.5  ], 
                                [0.0, 0.0, 0.0, 1.0  ]])
    
    T_ce_standoff   = np.array([[ -1.0, 0.0,  0.0, 0.0 ], 
                                [  0.0, 1.0,  0.0, 0.0 ], 
                                [  0.0, 0.0, -1.0, 0.1 ], 
                                [  0.0, 0.0,  0.0, 1.0 ]])
    
    T_ce_grasp      = np.array([[ -1.0, 0.0,  0.0, 0.0  ], 
                                [  0.0, 1.0,  0.0, 0.0  ], 
                                [  0.0, 0.0, -1.0, -0.025 ], 
                                [  0.0, 0.0,  0.0, 1.0  ]])
    k = 1
    
    # Calculate the reference trajectory for the end-effector to follow
    traj = mm.TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k=k, trajectory_opt="cartesian", time_scale_method="quintic")

    # ============================== The system's parameters and initial configuration ==============================
    # PID control gain: choosing between best and overshoot scenario
    if (data == "overshoot"):
        Kp = np.array([ [4, 0, 0, 0, 0, 0],
                        [0, 4, 0, 0, 0, 0],
                        [0, 0, 4, 0, 0, 0],
                        [0, 0, 0, 4, 0, 0],
                        [0, 0, 0, 0, 4, 0],
                        [0, 0, 0, 0, 0, 4]])
        
        Ki = np.array([[0.3, 0, 0, 0, 0, 0],
                        [0, 0.3, 0, 0, 0, 0],
                        [0, 0, 0.3, 0, 0, 0],
                        [0, 0, 0, 0.3, 0, 0],
                        [0, 0, 0, 0, 0.3, 0],
                        [0, 0, 0, 0, 0, 0.3]])  
    else:
        Kp = np.array([[2, 0, 0, 0, 0, 0],
                        [0, 2, 0, 0, 0, 0],
                        [0, 0, 2, 0, 0, 0],
                        [0, 0, 0, 2, 0, 0],
                        [0, 0, 0, 0, 2, 0],
                        [0, 0, 0, 0, 0, 2]])
        
        Ki = np.array([ [1e-5, 0, 0, 0, 0, 0],
                        [0, 1e-5, 0, 0, 0, 0],
                        [0, 0, 1e-5, 0, 0, 0],
                        [0, 0, 0, 1e-5, 0, 0],
                        [0, 0, 0, 0, 1e-5, 0],
                        [0, 0, 0, 0, 0, 1e-5]])  
    # Choose initial configuration of mobile manipulator
    if data == "newtask":
        q = np.array([0.5, -0.25, 0.25])
        u = np.array([0.0, 0.0, 0.0, 0.0])
        thetalist = np.array([0.5, 0.5, -0.3, 0.0, 0.1])
        current_cfg = np.concatenate((q, thetalist, u)) 
    else:
        q = np.array([0.0, 0.0, 0.0])
        u = np.array([0.0, 0.0, 0.0, 0.0])
        thetalist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        current_cfg = np.concatenate((q, thetalist, u))
    
    joint_max_speed =10
    wheel_max_speed =10
    
    # Mobile robot parameters
    l=0.235
    r=0.0475
    w=0.15
    
    # Robotic arm parameters
    Blist = np.array([[0.0,  0.0, 1.0,  0.0     , 0.033 , 0.0],
                      [0.0, -1.0, 0.0, -0.5076  , 0.0   , 0.0],
                      [0.0, -1.0, 0.0, -0.3526  , 0.0   , 0.0],
                      [0.0, -1.0, 0.0, -0.2176  , 0.0   , 0.0],
                      [0.0,  0.0, 1.0,  0.0     , 0.0   , 0.0]]).T

    T_b0 = np.array([[1.0, 0.0, 0.0, 0.1662 ],
                     [0.0, 1.0, 0.0, 0.0    ],
                     [0.0, 0.0, 1.0, 0.0026 ],
                     [0.0, 0.0, 0.0, 1.0    ]])
    
    M_0e = np.array([[1.0, 0.0, 0.0, 0.033  ],
                     [0.0, 1.0, 0.0, 0.0    ],
                     [0.0, 0.0, 1.0, 0.6546 ],
                     [0.0, 0.0, 0.0, 1.0    ]])
    
    joint_limit = np.array([ [-np.pi    , np.pi     ],
                             [-2        , 2         ],
                             [-2        , 2         ],
                             [-2        , 2         ],
                             [-np.pi    , np.pi     ]])
    
    
    # Initial configuration of end-effector
    X                   = mm.ForwardKinematics(current_cfg, T_b0, M_0e, Blist)
    time_step           = 0.01
    integral_error      = 0
    cfg_hist            = []
    current_cfg_save    = copy.deepcopy(current_cfg.tolist())
    current_cfg_save.append(0)
    cfg_hist.append(current_cfg_save)
    err_hist            = []
    v_dot_hist          = []

    with open(f"./{sub_dir}/{log_file}", "w", encoding="utf-8") as f:
        f.write("================================= Initial parameter =================================\n")
        f.write(f"- Initial configuration: {current_cfg}\n")
        f.write(f"- Controller: {controller}\n")
        f.write(f"- Proportional gains: \n")
        for i in range(Kp.shape[0]):
            f.write(f"{Kp[i]}\n")
        f.write(f"- Integral gain: \n")
        for i in range(Ki.shape[0]):
            f.write(f"{Ki[i]}\n")
        f.write("================================= Error tracking during manipulation =================================\n")
    
    # ============================== Feedforward Control Loop ==============================

    for i in range(len(traj) - 1):
        # Current end-effector configuration
        thetalist                   = np.array(current_cfg[3:8])
        X                           = mm.ForwardKinematics(current_cfg, T_b0, M_0e, Blist)
        # Desired end-effector configuration
        R_d                         = np.reshape(np.array(traj[i][0:9]), (3, 3))
        p_d                         = np.reshape(np.array(traj[i][9:12]), (3, 1))
        X_d                         = np.vstack((np.hstack((R_d, p_d)), np.array([0.0, 0.0, 0.0, 1.0])))
        # Next desired end-effector configuration
        R_dnext                     = np.reshape(np.array(traj[i+1][0:9]), (3, 3))
        p_dnext                     = np.reshape(np.array(traj[i+1][9:12]), (3, 1))
        X_dnext                     = np.vstack((np.hstack((R_dnext, p_dnext)), np.array([0.0, 0.0, 0.0, 1.0])))

        # Twist feedforward control law to calculate the control command
        V, X_err, integral_error    = mm.FeedforwardControl(X, X_d, X_dnext, Kp, Ki, time_step, integral_error, controller)
        
        # Actual control command to wheels and joints
        v_dot                       = mm.TwistToJointSpeed(V, M_0e, T_b0, Blist, thetalist, joint_max_speed, joint_limit, time_step=0.01, l=l, r=r, w=w)
        control_input               = v_dot.tolist()
        current_cfg                 = mm.NextState(current_cfg, control_input, time_step, [wheel_max_speed, joint_max_speed], l, r, w)
        # current_cfg[3:8]    = JointLimitTest(current_cfg[3:8], joint_limit)

        # Log the configuration and tracking error for analysis and visualization
        current_cfg_save            = copy.deepcopy(current_cfg)
        current_cfg_save.append(traj[i][-1])
        cfg_hist.append(current_cfg_save)
        err_hist.append(X_err)
        v_dot_hist.append(V)

        with open(f"./{sub_dir}/{log_file}", "a", encoding="utf-8") as f:
            f.write(f"Tracking error X_err: {X_err}\n")
            f.write(f"Control commands: {v_dot}\n")

        # Display log
        print("Tracking error X_err:", X_err)       
        print("Control commands:", v_dot)

    err_hist = np.array(err_hist)
    # Save to CSV file
    save_result(cfg_hist, data, f"{data}_cfg_{controller}_control")
    save_result(err_hist, data, "Tracking_error")
    save_result(v_dot_hist, data, "Control command")
    
    # plot tracking error
    plt.figure(1)
    plt.subplot(2, 1, 1)
    plt.plot(err_hist[:, 0], label='omega_x_err')
    plt.plot(err_hist[:, 1], label='omega_y_err')
    plt.plot(err_hist[:, 2], label='omega_z_err')
    plt.plot(err_hist[:, 3], label='v_x_err')
    plt.plot(err_hist[:, 4], label='v_y_err')
    plt.plot(err_hist[:, 5], label='v_z_err')
    plt.xlabel('Time step')
    plt.ylabel('Tracking Error')
    plt.title('Tracking Error over Time')
    plt.legend()


    # plot theta list to check joint limit and collision
    cfg_hist = np.array(cfg_hist)
    plt.subplot(2, 1, 2)
    plt.plot(cfg_hist[:, 3], label='theta_1')
    plt.plot(cfg_hist[:, 4], label='theta_2')
    plt.plot(cfg_hist[:, 5], label='theta_3')
    plt.plot(cfg_hist[:, 6], label='theta_4')
    plt.plot(cfg_hist[:, 7], label='theta_5')
    plt.xlabel('Time step')
    plt.ylabel('Joint angle')
    plt.title('Joint angle over Time')
    plt.legend()
    plt.savefig(f"./{sub_dir}/{log_fig}", dpi=300, bbox_inches='tight', transparent=False)
    plt.show()





if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--controller", help="Controller type: FPI, PI, F, P")
    parser.add_argument("-d", "--data", help="Choose initial configuration of mobile manipulator: best, overshoot or newtask")
    args = parser.parse_args()

    if args.data and args.controller:   
        main(args.data, args.controller)
    else:
        print("Not enough input argument !!!")

    # test()