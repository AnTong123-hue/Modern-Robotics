import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(T_se_inital, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k=1, time_step=0.01, 
                        time_scale_method="cubic", trajectory_opt="twist", 
                        t_to_init_standoff=10, t_to_init_grasp=2, t_close=0.63, t_back_init_standoff=2, 
                        t_to_final_standoff=10, t_to_final_grasp=2,t_open=0.63, t_back_final_standoff=2):
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

    # Trajectory 1: From T_se_initial to T_se_initial_standoff
    N = int(t_to_init_standoff / time_step) * k
    if trajectory_opt == "twist":
        traj1 = mr.ScrewTrajectory(T_se_inital, T_se_initial_standoff, t_to_init_standoff, N, time_scale_method)
    elif trajectory_opt == "cartesian":
        traj1 = mr.CartesianTrajectory(T_se_inital, T_se_initial_standoff, t_to_init_standoff, N, time_scale_method)
    else:
        raise ValueError("Invalid trajectory_opt. Must be 'twist' or 'cartesian'.")
    
    # Trajectory 2: From T_se_initial_standoff to T_se_initial_graps
    

    

    
def main():
    pass

if __name__ == "__main__":
    main()
