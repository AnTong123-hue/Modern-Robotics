import modern_robotics as mr
import numpy as np

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
    arm_cfg = current_cfg[3:9]
    wheel_cfg = current_cfg[9:]

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

def main():
    # Example usage of the NextState function
    current_cfg = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    control_input = [-10.0, 10.0, 10.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_step = 0.01
    max_speed = 12.3
    for i in range(101):
        current_cfg = NextState(current_cfg, control_input, time_step, max_speed)
        print(f"Time: {i*time_step:.2f}s, Configuration: {current_cfg}")

if __name__ == "__main__":
    main()



