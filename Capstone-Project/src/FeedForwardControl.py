import modern_robotics as mr
import numpy as np
import csv
import copy
import matplotlib.pyplot as plt

# Set precision to 3 decimal places
np.set_printoptions(precision=3)

def save_result(csvdata, filename, opt=1):
    with open(f"results/{filename}.csv", 'w', newline='') as save_file:
        writer = csv.writer(save_file)
        if (opt == 1):
            writer.writerows(csvdata)
        else:
            writer.writerow(csvdata)

def extract_traj(filename):
    traj_csv = []
    with open(f"results/{filename}.csv") as input_file:
        traj_csv_file = csv.reader(input_file)
        for line in traj_csv_file:
            if (line[0][0] != "#"):
                data = []
                for i in range(len(line)):
                    data.append(float(line[i]))
                traj_csv.append(data)
    return traj_csv

def NextState(current_cfg, control_input, time_step, max_speed, l=0.235, r=0.0475, w=0.15):
    """
    Arguments:
    - current_cfg: A tuple (chassis configuration(x, y, phi), arm configuration(theta1, theta2, theta3, theta4, theta5, theta6), wheel angles(u1, u2, u3, u4))
    - control_input: A tuple (wheel_speed(u1_dot, u2_dot, u3_dot, u4_dot), arm joint speed(theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot, theta6_dot))
    - time_step: The time step for the simulation
    - max_speed: The maximum speed for the wheels and arm joints

    Returns:
    - next_cfg: A tuple (chassis configuration(x, y, phi), arm configuration(theta0, theta1, theta2, theta3, theta4, theta5), wheel angles(u0, u1, u2, u3)) representing the next configuration of the robot after applying the control input for the given time step.
    """
    # Convert data to numpy arrays for easier manipulation
    current_cfg = np.array(current_cfg)
    control_input = np.array(control_input)

    # Separate the current configuration into chassis, arm and wheel components
    chassis_cfg = current_cfg[0:3]
    arm_cfg = current_cfg[3:8]
    wheel_cfg = current_cfg[8:]

    # Separate the control input intot wheel speed and arm joint speed
    wheel_speed = control_input[0:4]
    arm_joint_speed = control_input[4:]

    # Limit the wheel speed and arm joint speed to the maximum speed
    wheel_speed = np.clip(wheel_speed, -max_speed, max_speed)
    arm_joint_speed = np.clip(arm_joint_speed, -max_speed, max_speed)

    # Update the wheel angles and arm joint angles based on the control inpu tand time step
    next_wheel_cfg = wheel_cfg + wheel_speed * time_step
    next_arm_cfg = arm_cfg + arm_joint_speed * time_step 

    # Compute the change in chassis configuration based on the wheel speeds
    delta_wheel_cfg = wheel_speed * time_step
    odometry_mat = np.array([[-r/(4*(l+w))  , r/(4*(l+w))   ,  r/(4*(l+w))  , -r/(4*(l+w))   ],
                             [ r/4          , r/4           ,  r/4          ,  r/4          ],
                             [-r/4          , r/4           , -r/4          ,  r/4          ]])
    body_twist_3 = np.dot(odometry_mat, delta_wheel_cfg)
    body_twist_6 = np.concatenate((np.zeros(2), body_twist_3, np.zeros(1)))
    se3_mat = mr.VecTose3(body_twist_6)
    tf_mat = mr.MatrixExp6(se3_mat)
    chassis_cfg_mat = np.array([[np.cos(chassis_cfg[0]) , -np.sin(chassis_cfg[0]), 0, chassis_cfg[1]],
                                [np.sin(chassis_cfg[0]) ,  np.cos(chassis_cfg[0]), 0, chassis_cfg[2]],
                                [0                      ,  0                     , 1, 0             ],
                                [0                      ,  0                     , 0, 1             ]])
    next_chassis_cfg_mat = np.dot(chassis_cfg_mat, tf_mat)
    next_chassis_cfg = np.array([np.arctan2(next_chassis_cfg_mat[1,0], next_chassis_cfg_mat[0,0]), 
                                 next_chassis_cfg_mat[0,3], 
                                 next_chassis_cfg_mat[1,3]])
    
    # Combine the next chassis configuration, arm configuration and wheel angles into a single tuple
    next_cfg = np.concatenate((next_chassis_cfg, next_arm_cfg, next_wheel_cfg))
    next_cfg = next_cfg.tolist()  # Convert back to list for output
    return next_cfg

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
        # traj = traj.tolist()

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
        # traj = traj.tolist()
    else:
        raise ValueError("Invalid trajectory_opt. Must be 'twist' or 'cartesian'.")
    
    # Save the trajectory to a CSV file
    save_result(traj, "reference_trajectory", opt=1)
    
    return traj

def MMForwardKinematics(current_cfg, T_b0, M_0e, Blist):
    phi, x, y = current_cfg[0:3]
    thetalist = current_cfg[3:8]
    wheel_cfg = current_cfg[8:]

    # From space frame to mobile frame
    T_sb = np.array([   [np.cos(phi), -np.sin(phi), 0.0, x      ],
                        [np.sin(phi),  np.cos(phi), 0.0, y      ],
                        [0.0        , 0.0         , 1.0, 0.0963 ],
                        [0.0        , 0.0         , 0.0, 1.0    ]])
    # From base 0 to end-effector
    T_0e = mr.FKinBody(M_0e, Blist, thetalist)

    # From space frame to end-effector frame
    T_se = T_sb @ T_b0 @ T_0e
    return T_se

def FeedforwardControl(X, X_d, X_dnext, Kp, Ki, time_step=0.01, last_integral_error=np.zeros(6)):
    """
    Feedforward control law for trajectory tracking.
    Arguments:
    - X         : A numpy array of shape (4, 4) representing the current end-effector configuration.
    - Xd        : A numpy array of shape (4, 4) representing the desired end-effector configuration at the current time stamps.
    - X_dnext:  : A numpy array of shape (4, 4) representing the next desired end-effector configuration at the next time stamps.
    - Kp        : A numpy array of shape (6, 6) representing the proportional gain 
    - Ki        : A numpy array of shape (6, 6) representing the integral gain
    - time_step : A float representing the time step for numerical integration (default is 0.01 seconds).

    Returns:
    - V         : A numpy array of shape (6,) representing the commanded end-effector twist. 
    - X_err     : A numpy array of shape (6,) representing the current tracking error in se3 vector form.
    - integral_error : A numpy array of shape (6,) representing the integral of the tracking error up to the current time step.
    """ 

    # Tracking error
    X_err_se3  = mr.MatrixLog6(mr.TransInv(X) @ X_d)
    X_err      = mr.se3ToVec(X_err_se3)

    print("Tracking error X_err:", X_err)

    # Desired end-effector twist
    V_d_se3    = 1 / time_step * mr.MatrixLog6(mr.TransInv(X_d) @ X_dnext)
    V_d        = mr.se3ToVec(V_d_se3)
    # print("Desired end-effector twist V_d:")
    # print(V_d)

    # End-effector twist in body frame
    V_d_body   = mr.Adjoint(mr.TransInv(X) @ X_d) @ V_d
    # print("Desired end-effector twist in body frame V_d_body:")
    # print(V_d_body)

    # Integral of the error
    X_err_integral = last_integral_error + X_err * time_step

    # Feedforward control law
    V = V_d_body + Kp @ X_err + Ki @ X_err_integral

    return V, X_err, X_err_integral

def TestJointLimit(current_joint, joint_speed, max_speed, joint_limit, time_step=0.01):
    """
    Test whether next configuration of robotic violates joints' limit
    Arguments:
    - current_joint   : A numpy array of shape (5, ) representing current joint angles
    - joint_speed     : A numpy array of shape (6, ) representing current joint speeds
    - max_speed       : A scalar represent maximumm speed of each joint
    - joint_limit     : A numpy array of shape (6, ) representing joint limit values
    - time_step       : A float representing the time step for numerical integration (default is 0.01 seconds).

    Results:
    - joint_violates  : A binary numpy array of shape (5, ) representing the joints violating their limits
    """
    joint_speed = np.clip(joint_speed, -max_speed, max_speed)
    next_joint = current_joint + joint_speed * time_step
    joint_violates = np.abs(next_joint) > joint_limit

    return joint_violates

def JointLimitTest(current_joint, joint_limit):
    next_joint = copy.deepcopy(current_joint)
    for i in range(len(current_joint)):
        if current_joint[i] > joint_limit[i]:
            next_joint[i] = joint_limit[i]
        elif current_joint[i] < -joint_limit[i]:
            next_joint[i] = -joint_limit[i]
        else:
            next_joint[i] = current_joint[i]
    return next_joint

def TwistToJointSpeed(V, M_0e, T_b0, Blist, thetalist, max_speed=10, joint_limit=np.zeros(5), time_step = 0.01, l=0.235, r=0.0475, w=0.15):
    """
    Calculate wheel speed and joint speed from control twist 
    Argumemts:
    - V         : A numpy array of shape (6,) representing the commanded end-effector twist. 
    - M_0e      : A numpy array of shape (4, 4) representing the initial configuration of end-effector
    - T_b0      : A numpy array of shape (4, 4) representing the configuration of robotic arm base relative to mobile base
    - Blist     : A numpy array of shape (6, 5) representing the twist of each joint on robotic arm
    - thetalist : A numpy array of shape (5,) representing joint angles

    Return:
    - v_dot     : A numpy array of shape (9, ) representing the commanded wheel speed and joint speed
    """ 
    J_e = mr.JacobianBody(Blist, thetalist)

    T_0e = mr.FKinBody(M_0e, Blist, thetalist)

    odometry_mat = np.array([[-r/(4*(l+w))  , r/(4*(l+w))   ,  r/(4*(l+w))  , -r/(4*(l+w))   ],
                            [ r/4          , r/4           ,  r/4          ,  r/4          ],
                            [-r/4          , r/4           , -r/4          ,  r/4          ]])
    odometry_mat_6 = np.vstack((np.zeros((2,4)), odometry_mat, np.zeros((1,4))))
    T_eb = mr.TransInv(T_0e) @ mr.TransInv(T_b0)
    J_base = mr.Adjoint(T_eb) @ odometry_mat_6

    J_total = np.hstack((J_base, J_e))
    print("Jacobian matrix")
    print(J_total)
    v_dot = np.linalg.pinv(J_total, rtol=0.001) @ V
    return v_dot
    joint_violates = TestJointLimit(thetalist, v_dot[4:], max_speed, joint_limit, time_step)

    while (np.sum(joint_violates) != 0):
        J_e_copy = copy.deepcopy(J_e)
        for i in range(joint_violates.shape[0]):
            J_e_copy[:, i] = J_e_copy[:, i] * (1 - joint_violates[i])
        J_total = np.hstack((J_base, J_e_copy))
        v_dot = np.linalg.pinv(J_total) @ V
        joint_violates = TestJointLimit(thetalist, v_dot[4:], max_speed, joint_limit, time_step)

    return v_dot


def main():
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
    # Calculate the reference trajectory for the end-effector to follow
    traj = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k=k, trajectory_opt="cartesian")

    # ============================== The system's parameters and initial configuration ==============================
    # PID control gain
    # Kp = np.zeros((6,6))
    Kp = np.array([[1, 0, 0, 0, 0, 0],
                   [0, 1, 0, 0, 0, 0],
                   [0, 0, 1, 0, 0, 0],
                   [0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 1]])
    
    Ki = np.array([[0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0]])  
    
    max_speed = 10
    
    # Mobile robot parameters
    l=0.235
    r=0.0475
    w=0.15
    
    q = np.array([0.0, 0.0, 0.0])
    u = np.array([0.0, 0.0, 0.0, 0.0])

    # Robotic arm parameters
    thetalist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    Blist = np.array([[0.0, 0.0, 1.0, 0.0, 0.033, 0.0],
                      [0.0, -1.0, 0.0, -0.5076, 0.0, 0.0],
                      [0.0, -1.0, 0.0, -0.3526, 0.0, 0.0],
                      [0.0, -1.0, 0.0, -0.2176, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
    
    T_b0 = np.array([[1.0, 0.0, 0.0, 0.1662],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0026],
                     [0.0, 0.0, 0.0, 1.0]])
    
    M_0e = np.array([[1.0, 0.0, 0.0, 0.033],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.6546],
                     [0.0, 0.0, 0.0, 1.0]])
    
    current_cfg = np.concatenate((q, thetalist, u))

    # Initial configuration of end-effector
    X       = MMForwardKinematics(current_cfg, T_b0, M_0e, Blist)
    time_step = 0.01
    integral_error = 0
    cfg_hist = []
    current_cfg_save    = copy.deepcopy(current_cfg.tolist())
    current_cfg_save.append(0)
    print(len(current_cfg_save))
    cfg_hist.append(current_cfg_save)
    err_hist = []

    for i in range(len(traj) - 1):
        # Current end-effector configuration
        X           = MMForwardKinematics(current_cfg, T_b0, M_0e, Blist)
        # Desired end-effector configuration
        R_d         = np.reshape(np.array(traj[i][0:9]), (3, 3))
        p_d         = np.reshape(np.array(traj[i][9:12]), (3, 1))
        X_d         = np.vstack((np.hstack((R_d, p_d)), np.array([0.0, 0.0, 0.0, 1.0])))
        # Next desired end-effector configuration
        R_dnext     = np.reshape(np.array(traj[i+1][0:9]), (3, 3))
        p_dnext     = np.reshape(np.array(traj[i+1][9:12]), (3, 1))
        X_dnext     = np.vstack((np.hstack((R_dnext, p_dnext)), np.array([0.0, 0.0, 0.0, 1.0])))

        # Twist feedforward control law to calculate the control command
        V, X_err, integral_error   = FeedforwardControl(X, X_d, X_dnext, Kp, Ki, time_step, integral_error)
        
        # Actual control command to wheels and joints
        v_dot               = TwistToJointSpeed(V, M_0e, T_b0, Blist, thetalist, max_speed, np.array([1.6, 1.6, 0.2, 1.6, 1.6]), time_step=0.01, l=l, r=r, w=w)
        print("Control commands:", v_dot)
        control_input       = v_dot.tolist()
        current_cfg         = NextState(current_cfg, control_input, time_step, max_speed, l, r, w)
        current_cfg[3:8]   = JointLimitTest(current_cfg[3:8], np.array([2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi]))

        # Log the configuration and tracking error for analysis and visualization
        current_cfg_save    = copy.deepcopy(current_cfg)
        current_cfg_save.append(traj[i][-1])
        cfg_hist.append(current_cfg_save)
        err_hist.append(X_err)
    
    # plot tracking error
    plt.figure()
    err_hist = np.array(err_hist)
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
    plt.show()
    save_result(cfg_hist, "control")

def test():

    print("Test feedforward control law")
    thetalist = np.array([0.0, 0.0, 0.2, -1.6, 0.0])

    Blist = np.array([[0.0, 0.0, 1.0, 0.0, 0.033, 0.0],
                      [0.0, -1.0, 0.0, -0.5076, 0.0, 0.0],
                      [0.0, -1.0, 0.0, -0.3526, 0.0, 0.0],
                      [0.0, -1.0, 0.0, -0.2176, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
    
    T_b0 = np.array([[1.0, 0.0, 0.0, 0.1662],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0026],
                     [0.0, 0.0, 0.0, 1.0]])
    
    M_0e = np.array([[1.0, 0.0, 0.0, 0.033],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.6546],
                     [0.0, 0.0, 0.0, 1.0]])
    
    X_d = np.array([[0.0, 0.0, 1.0, 0.5],
                    [0.0, 1.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0, 0.5],
                    [0.0, 0.0, 0.0, 1.0]])
    
    X_dnext = np.array([[0.0, 0.0, 1.0, 0.6],
                        [0.0, 1.0, 0.0, 0.0], 
                        [-1.0, 0.0, 0.0, 0.3],
                        [0.0, 0.0, 0.0, 1.0]])

    X = np.array([[0.170, 0.0, 0.985, 0.387],
                    [0.0, 1.0, 0.0, 0.0], 
                    [-0.985, 0.0, 0.170, 0.570],
                    [0.0 ,0.0, 0.0, 1.0]])
    
    V, _, _ = FeedforwardControl(X, X_d, X_dnext, np.eye(6), np.zeros((6,6)))
    control_cmd = TwistToJointSpeed(V, M_0e, T_b0, Blist, thetalist)
    print("Control twist: ", V)
    print("Control command: ", control_cmd)
    
    print("Test forward kinematics")
    q = np.array([0.0, 0.0, 0.0])
    u = np.array([0.0, 0.0, 0.0, 0.0])
    thetalist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    
    current_cfg = [0, 0, 0, 0, 0, 0.2,-1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    T_se = MMForwardKinematics(current_cfg, T_b0, M_0e, Blist)
    print("End-effector initial configuration")
    print(T_se)

    # Test trajectory generation
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

    traj = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k=1, trajectory_opt="cartesian")
    initial_cfg = traj[0]
    print("Initial configuration of the trajectory: ", initial_cfg)
    R_d = np.reshape(np.array(initial_cfg[0:9]), (3, 3))
    p_d = np.reshape(np.array(initial_cfg[9:12]), (3, 1))
    X_d = np.vstack((np.hstack((R_d, p_d)), np.array([0.0, 0.0, 0.0, 1.0])))
    print("Initial configuration of the trajectory in SE(3): ")
    print(X_d)

if __name__ == "__main__":
    main()
    # test()