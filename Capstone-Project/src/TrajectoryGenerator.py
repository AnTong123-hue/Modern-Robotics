import numpy as np
import modern_robotics as mr
import csv

def save_result(csvdata, filename, opt=1):
    with open(f"results/{filename}.csv", 'w', newline='') as save_file:
        writer = csv.writer(save_file)
        if (opt == 1):
            writer.writerows(csvdata)
        else:
            writer.writerow(csvdata)

def TrajectoryGenerator(T_se_inital, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k=1, time_step=0.01, 
                        time_scale_method="cubic", trajectory_opt="twist", 
                        t_to_init_standoff=10, t_to_init_grasp=2, t_close=1, t_back_init_standoff=2, 
                        t_to_final_standoff=10, t_to_final_grasp=2,t_open=1, t_back_final_standoff=2):
    """
    Arguments:
    - T_se_initial: np.array of shape (4, 4) representing the initial end-effector configuration in {s} frame
    - T_sc_initial: np.array of shape (4, 4) representing the initial object configuration in {s} frame
    - T_sc_final: np.array of shape (4, 4) representing the final object configuration in {s} frame
    - T_ce_grasp: np.array of shape (4, 4) representing the end-effector configuration relative to the object when grasping
    - T_ce_standoff: np.array of shape (4, 4) representing the end-effector configuration relative to the object when in standoff position
    - k: The number of trajectory points per 0.01 seconds
    - time_step: The time step for the trajectory generation
    - trajectory_opt: "twist" for twist-based trajectory generation, "cartesian" for rotaion and translation decoupled trajectory generation
    - time_scale_method: "cubic" for cubic time scaling, "quintic" for quintic time scaling
    - t_*: The time durations for each segment of the trajectory in seconds

    Returns:
    - traj: A matrix of N flatten configuration references of shape (N, 13) representing the end-effector configurations at each time step along the trajectory
    Example return value: [r_11, r_12, r_13, r_21, r_22, r_23, r_31, r_32, r_33, p_x, p_y, p_z, gripper_state]

    Additional Notes:
    - A csv file with eight-segment reference trajectory should be generated 
    - Opening and closing the gripper takes up to 0.625 seconds
    """
    T_se_initial_standoff = T_sc_initial @ T_ce_standoff
    T_se_initial_grasp = T_sc_initial @ T_ce_grasp
    T_se_final_standoff = T_sc_final @ T_ce_standoff   
    T_se_final_grasp = T_sc_final @ T_ce_grasp

    N1 = int(t_to_init_standoff / time_step) * k
    N2 = int(t_to_init_grasp / time_step) * k
    N3 = int(t_close / time_step) * k
    N4 = int(t_back_init_standoff / time_step) * k
    N5 = int(t_to_final_standoff / time_step) * k
    N6 = int(t_to_final_grasp / time_step) * k
    N7 = int(t_open / time_step) * k
    N8 = int(t_back_final_standoff / time_step) * k

    if trajectory_opt == "twist":
        # Trajectory 1: From T_se_initial to T_se_initial_standoff
        traj1       = mr.ScrewTrajectory(T_se_inital, T_se_initial_standoff, t_to_init_standoff, N1, time_scale_method)
        traj1_rot   = np.array(traj1)[:, :3, :3]
        traj1_trans = np.array(traj1)[:, :3, 3]
        traj1_stack = np.hstack((traj1_rot.reshape(traj1_rot.shape[0],-1), traj1_trans.reshape(traj1_trans.shape[0], -1), np.zeros((N1, 1))))
        # Trajectory 2: From T_se_initial_standoff to T_se_initial_grasp
        traj2       = mr.ScrewTrajectory(T_se_initial_standoff, T_se_initial_grasp, t_to_init_grasp, N2, time_scale_method)
        traj2_rot   = np.array(traj2)[:, :3, :3]
        traj2_trans = np.array(traj2)[:, :3, 3]
        traj2_stack = np.hstack((traj2_rot.reshape(traj2_rot.shape[0],-1), traj2_trans.reshape(traj2_trans.shape[0], -1), np.zeros((N2, 1))))
        # Trajectory 3: Gripper closing (hold the end-effector configuration constant)
        traj3_stack = np.tile(np.hstack((T_se_initial_grasp[:3, :3].flatten(), T_se_initial_grasp[:3, 3], np.array([1]))), (N3, 1))
        # Trajectory 4: From T_se_initial_grasp back to T_se_initial_standoff
        traj4       = mr.ScrewTrajectory(T_se_initial_grasp, T_se_initial_standoff, t_back_init_standoff, N4, time_scale_method)
        traj4_rot   = np.array(traj4)[:, :3, :3]
        traj4_trans = np.array(traj4)[:, :3, 3]
        traj4_stack = np.hstack((traj4_rot.reshape(traj4_rot.shape[0],-1), traj4_trans.reshape(traj4_trans.shape[0], -1), np.ones((N4, 1))))
        # Trajectory 5: From T_se_initial_standoff to T_se_final_standoff
        traj5       = mr.ScrewTrajectory(T_se_initial_standoff, T_se_final_standoff, t_to_final_standoff, N5, time_scale_method)
        traj5_rot   = np.array(traj5)[:, :3, :3]
        traj5_trans = np.array(traj5)[:, :3, 3]
        traj5_stack = np.hstack((traj5_rot.reshape(traj5_rot.shape[0],-1), traj5_trans.reshape(traj5_trans.shape[0], -1), np.ones((N5, 1))))
        # Trajectory 6: From T_se_final_standoff to T_se_final_grasp
        traj6       = mr.ScrewTrajectory(T_se_final_standoff, T_se_final_grasp, t_to_final_grasp, N6, time_scale_method)
        traj6_rot   = np.array(traj6)[:, :3, :3]
        traj6_trans = np.array(traj6)[:, :3, 3]
        traj6_stack = np.hstack((traj6_rot.reshape(traj6_rot.shape[0],-1), traj6_trans.reshape(traj6_trans.shape[0], -1), np.ones((N6, 1))))
        # Trajectory 7: Gripper opening (hold the end-effector configuration constant)
        traj7_stack = np.tile(np.hstack((T_se_final_grasp[:3, :3].flatten(), T_se_final_grasp[:3, 3], np.array([0]))), (N7, 1))
        # Trajectory 8: From T_se_final_grasp back to T_se_final_standoff
        traj8       = mr.ScrewTrajectory(T_se_final_grasp, T_se_final_standoff, t_back_final_standoff, N8, time_scale_method)
        traj8_rot   = np.array(traj8)[:, :3, :3]
        traj8_trans = np.array(traj8)[:, :3, 3]
        traj8_stack = np.hstack((traj8_rot.reshape(traj8_rot.shape[0],-1), traj8_trans.reshape(traj8_trans.shape[0], -1), np.zeros((N8, 1))))

        # Vertically stack all trajectory segments to form the complete trajectory
        traj = np.vstack((traj1_stack, traj2_stack, traj3_stack, traj4_stack, traj5_stack, traj6_stack, traj7_stack, traj8_stack))
        # Convert the trajectory to a list of lists for easier handling
        traj.tolist()

    elif trajectory_opt == "cartesian":
        # Trajectory 1: From T_se_initial to T_se_initial_standoff
        traj1       = mr.CartesianTrajectory(T_se_inital, T_se_initial_standoff, t_to_init_standoff, N1, time_scale_method)
        traj1_rot   = np.array(traj1)[:, :3, :3]
        traj1_trans = np.array(traj1)[:, :3, 3]
        traj1_stack = np.hstack((traj1_rot.reshape(traj1_rot.shape[0],-1), traj1_trans.reshape(traj1_trans.shape[0], -1), np.zeros((N1, 1))))
        # Trajectory 2: From T_se_initial_standoff to T_se_initial_grasp
        traj2       = mr.CartesianTrajectory(T_se_initial_standoff, T_se_initial_grasp, t_to_init_grasp, N2, time_scale_method)
        traj2_rot   = np.array(traj2)[:, :3, :3]
        traj2_trans = np.array(traj2)[:, :3, 3]
        traj2_stack = np.hstack((traj2_rot.reshape(traj2_rot.shape[0],-1), traj2_trans.reshape(traj2_trans.shape[0], -1), np.zeros((N2, 1))))
        # Trajectory 3: Gripper closing (hold the end-effector configuration constant)
        traj3_stack = np.tile(np.hstack((T_se_initial_grasp[:3, :3].flatten(), T_se_initial_grasp[:3, 3], np.array([1]))), (N3, 1))
        # Trajectory 4: From T_se_initial_grasp back to T_se_initial_standoff
        traj4       = mr.CartesianTrajectory(T_se_initial_grasp, T_se_initial_standoff, t_back_init_standoff, N4, time_scale_method)
        traj4_rot   = np.array(traj4)[:, :3, :3]
        traj4_trans = np.array(traj4)[:, :3, 3]
        traj4_stack = np.hstack((traj4_rot.reshape(traj4_rot.shape[0],-1), traj4_trans.reshape(traj4_trans.shape[0], -1), np.ones((N4, 1))))
        # Trajectory 5: From T_se_initial_standoff to T_se_final_standoff
        traj5       = mr.CartesianTrajectory(T_se_initial_standoff, T_se_final_standoff, t_to_final_standoff, N5, time_scale_method)
        traj5_rot   = np.array(traj5)[:, :3, :3]
        traj5_trans = np.array(traj5)[:, :3, 3]
        traj5_stack = np.hstack((traj5_rot.reshape(traj5_rot.shape[0],-1), traj5_trans.reshape(traj5_trans.shape[0], -1), np.ones((N5, 1))))
        # Trajectory 6: From T_se_final_standoff to T_se_final_grasp
        traj6       = mr.CartesianTrajectory(T_se_final_standoff, T_se_final_grasp, t_to_final_grasp, N6, time_scale_method)
        traj6_rot   = np.array(traj6)[:, :3, :3]
        traj6_trans = np.array(traj6)[:, :3, 3]
        traj6_stack = np.hstack((traj6_rot.reshape(traj6_rot.shape[0],-1), traj6_trans.reshape(traj6_trans.shape[0], -1), np.ones((N6, 1))))
        # Trajectory 7: Gripper opening (hold the end-effector configuration constant)
        traj7_stack = np.tile(np.hstack((T_se_final_grasp[:3, :3].flatten(), T_se_final_grasp[:3, 3], np.array([0]))), (N7, 1))
        # Trajectory 8: From T_se_final_grasp back to T_se_final_standoff
        traj8       = mr.CartesianTrajectory(T_se_final_grasp, T_se_final_standoff, t_back_final_standoff, N8, time_scale_method)
        traj8_rot   = np.array(traj8)[:, :3, :3]
        traj8_trans = np.array(traj8)[:, :3, 3]
        traj8_stack = np.hstack((traj8_rot.reshape(traj8_rot.shape[0],-1), traj8_trans.reshape(traj8_trans.shape[0], -1), np.zeros((N8, 1))))

        # Vertically stack all trajectory segments to form the complete trajectory
        traj = np.vstack((traj1_stack, traj2_stack, traj3_stack, traj4_stack, traj5_stack, traj6_stack, traj7_stack, traj8_stack))
        # Convert the trajectory to a list of lists for easier handling
        traj.tolist()
    else:
        raise ValueError("Invalid trajectory_opt. Must be 'twist' or 'cartesian'.")
    
    # Save the trajectory to a CSV file
    save_result(traj, "reference_trajectory", opt=1)
    
    return traj
    
def main():
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
                                [-1.0, 0.0, 0.0, 0.5 ], 
                                [0.0, 0.0, 0.0, 1.0  ]])
    
    T_ce_standoff   = np.array([[ 0.0, 0.0, 1.0, 0.0 ], 
                                [ 0.0, 1.0, 0.0, 0.0 ], 
                                [-1.0, 0.0, 0.0, 0.1 ], 
                                [ 0.0, 0.0, 0.0, 1.0 ]])
    
    T_ce_grasp      = np.array([[ 0.0, 0.0, 1.0, 0.0   ], 
                                [ 0.0, 1.0, 0.0, 0.0   ], 
                                [-1.0, 0.0, 0.0, 0.025 ], 
                                [ 0.0, 0.0, 0.0, 1.0   ]])
    k = 1
    
    traj = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k=k, trajectory_opt="cartesian")
    for i in range(len(traj)):
        print(f"Time {i*0.01/k}: {traj[i]}\n")


if __name__ == "__main__":
    main()
