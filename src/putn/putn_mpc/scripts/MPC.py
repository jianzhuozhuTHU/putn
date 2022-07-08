import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import time


def MPC(self_state, goal_state, obstacles):
    opti = ca.Opti()
    ## parameters for optimization
    T = 0.2
    N = 10  # MPC horizon
    v_max = 0.5
    omega_max = 0.6
    safe_distance = 0.55
    Q = np.array([[1.2, 0.0, 0.0],[0.0, 1.2, 0.0],[0.0, 0.0, 0.0]])
    R = np.array([[0.2, 0.0], [0.0, 0.15]])
    goal = goal_state[:,:3]
    tra = goal_state[:,2]
    conf = goal_state[:,3]
    tra_mean = np.mean(tra)
    opt_x0 = opti.parameter(3)
    opt_controls = opti.variable(N, 2)
    v = opt_controls[:, 0]
    omega = opt_controls[:, 1]

    ## state variables
    opt_states = opti.variable(N+1, 3)
    x = opt_states[:, 0]
    y = opt_states[:, 1]
    theta = opt_states[:, 2]

    ## create funciont for F(x)
    theta_x = self_state[0][4]*np.cos(self_state[0][2]) - self_state[0][3]*np.sin(self_state[0][2])
    theta_y = self_state[0][4]*np.sin(self_state[0][2]) + self_state[0][3]*np.cos(self_state[0][2])
    f = lambda x_, u_: ca.vertcat(*[u_[0]*ca.cos(x_[2])*np.cos(theta_x), u_[0]*ca.sin(x_[2])*np.cos(theta_y), u_[1]])

    ## init_condition
    opti.subject_to(opt_states[0, :] == opt_x0.T)

    # Position Boundaries
    # Here you can customize the avoidance of local obstacles 

    # Admissable Control constraints
    opti.subject_to(opti.bounded(-v_max, v, v_max))
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max)) 

    # System Model constraints
    for i in range(N):
        x_next = opt_states[i, :] + T*f(opt_states[i, :], opt_controls[i, :]).T
        opti.subject_to(opt_states[i+1, :]==x_next)

    #### cost function
    obj = 0 
    for i in range(N):
        obj = obj + 0.1*ca.mtimes([(opt_states[i, :] - goal[[i]]), Q, (opt_states[i, :]- goal[[i]]).T]) + ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T]) 
    obj = obj + 2*ca.mtimes([(opt_states[N-1, :] - goal[[N-1]]), Q, (opt_states[N-1, :]- goal[[N-1]]).T])

    opti.minimize(obj)
    opts_setting = {'ipopt.max_iter':80, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-3, 'ipopt.acceptable_obj_change_tol':1e-3}
    opti.solver('ipopt',opts_setting)
    opti.set_value(opt_x0, self_state[:,:3])

    try:
        sol = opti.solve()
        u_res = sol.value(opt_controls)
        state_res = sol.value(opt_states)
    except:
        state_res = np.repeat(self_state[:3],N+1,axis=0)
        u_res = np.zeros([N,2])

    return state_res, u_res
