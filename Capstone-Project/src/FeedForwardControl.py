import modern_robotics as mr
import numpy as np
# Set precision to 3 decimal places
np.set_printoptions(precision=3)
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
    """ 

    # Tracking error
    X_err_se3  = mr.MatrixLog6(mr.TransInv(X) @ X_d)
    X_err      = mr.se3ToVec(X_err_se3)

    print("Tracking error X_err:")
    print(X_err)

    # Desired end-effector twist
    V_d_se3    = 1 / time_step * mr.MatrixLog6(mr.TransInv(X_d) @ X_dnext)
    V_d        = mr.se3ToVec(V_d_se3)
    print("Desired end-effector twist V_d:")
    print(V_d)

    # End-effector twist in body frame
    V_d_body   = mr.Adjoint(mr.TransInv(X) @ X_d) @ V_d
    print("Desired end-effector twist in body frame V_d_body:")
    print(V_d_body)

    # Integral of the error
    X_err_integral = last_integral_error + X_err * time_step

    # Feedforward control law
    V = V_d_body + Kp @ X_err + Ki @ X_err_integral

    return V, X_err_integral


def test():
    X = np.array ([[0.17, 0, 0.985, 0.387  ], 
                   [0.0, 1.0, 0.0, 0.0     ], 
                   [-0.985, 0.0, 0.17, 0.57],
                   [0.0, 0.0, 0.0, 1.0     ]])
    
    X_d = np.array([[0.0, 0.0, 1.0, 0.5],
                    [0.0, 1.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0, 0.5],
                    [0.0, 0.0, 0.0, 1.0]])
    
    X_dnext = np.array([[0.0, 0.0, 0.0, 0.6],
                        [0.0, 1.0, 0.0, 0.0],
                        [-1.0, 0.0, 0.0, 0.3],
                        [0.0, 0.0, 0.0, 1.0]])
    
    Kp = np.zeros((6, 6))
    Ki = np.zeros((6, 6))  

    time_step = 0.01
    last_integral_error = np.zeros(6)

    V, integral_error = FeedforwardControl(X, X_d, X_dnext, Kp, Ki, time_step, last_integral_error)

    theta_list = np.array([0.0, 0.0, 0.2, -1.6, 0.0])
    Blist = np.array([[0.0, 0.0, 1.0, 0.0, 0.033, 0.0],
                      [0.0, -1.0, 0.0, -0.5076, 0.0, 0.0],
                      [0.0, -1.0, 0.0, -0.3526, 0.0, 0.0],
                      [0.0, -1.0, 0.0, -0.2176, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]]).T
    
    l=0.235
    r=0.0475
    w=0.15
    T_b0 = np.array([[1.0, 0.0, 0.0, 0.1662],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0026],
                     [0.0, 0.0, 0.0, 1.0]])
    
    M_0e = np.array([[1.0, 0.0, 0.0, 0.033],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.6546],
                     [0.0, 0.0, 0.0, 1.0]])
    
    J_e = mr.JacobianBody(Blist, theta_list)

    T_0e = mr.FKinBody(M_0e, Blist, theta_list)

    odometry_mat = np.array([[-r/(4*(l+w))  , r/(4*(l+w))   ,  r/(4*(l+w))  , -r/(4*(l+w))   ],
                            [ r/4          , r/4           ,  r/4          ,  r/4          ],
                            [-r/4          , r/4           , -r/4          ,  r/4          ]])
    odometry_mat_6 = np.vstack((np.zeros((2,4)), odometry_mat, np.zeros((1,4))))
    T_eb = mr.TransInv(T_0e) @ mr.TransInv(T_b0)
    J_base = mr.Adjoint(T_eb) @ odometry_mat_6
    J_total = np.hstack((J_base, J_e))  

    print("Jacobian J_total:")
    print(J_total)

    control_input = np.linalg.pinv(J_total) @ V
    print("Control input (wheel speeds and joint velocities):")
    print(control_input)

    print("Commanded end-effector twist V:")
    print(V)

if __name__ == "__main__":
    test()