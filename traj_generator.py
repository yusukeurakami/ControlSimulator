import numpy as np
import env
import scipy.linalg as la

def sin_traj(time_step, init):
    np.linspace(0,10,time_step)
    traj = []
    for i in range(time_step):
        traj.append([i/50.0, 0.0, 5.0*np.sin(i/50.0), 0.0])
    return np.array(traj)

def line_traj(time_step, start, goal):
    step_dist = (goal - start)/time_step
    traj = []
    for i in range(time_step):
        traj.append(start+step_dist*i)
    return np.array(traj)

# def min_snap_traj(time_step, start, goal):
#     ed = time_step * env.samp_t
#     mid_t = ed / 2.0
#     mid = np.array([[30.0, 50.0, 5.0, 50.0]]).T

#     #make the polynomial that satisfy following

#     # a3*t^3 + a2*t^2 + a1*t^1 + a0*t^0 = x
#     # 3*a3*t^2 + 2*a2*t^1 + a1*t^0 = x'
#     # 6*a3*t^1 + 2*a2*t^0 = x''

#     # Ax = B
#     # A 9x3
#     # x 3x2
#     # B 9x2

#     # [  0^3,   0^2, 0^1, 0^0]                                [start_pos_x, start_pos_y]
#     # [3*0^2, 2*0^1, 0^0,   0]                                [start_vel_x, start_vel_y]
#     # [6*0^1, 2*0^0,   0,   0]                   [x4, y4]     [start_acc_x, start_acc_y]
#     # [  mid_t^3,   mid_t^2, mid_t^1, mid_t^0]   [x3, y3]     [mid_pos_x, mid_pos_y]
#     # [3*mid_t^2, 2*mid_t^1, mid_t^0,       0] * [x2, y2]  =  [mid_vel_x, mid_vel_y]
#     # [6*mid_t^1, 2*mid_t^0,       0,       0]   [x1, y1]     [mid_acc_x, mid_acc_y]
#     # [  ed^3,   ed^2, ed^1, ed^0]                            [goal_pos]
#     # [3*ed^2, 2*ed^1, ed^0,    0]                            [goal_vel]
#     # [6*ed^1, 2*ed^0,    0,    0]                            [goal_acc]
    

#     A = np.array([[  0**3,   0**2, 0**1, 0**0],
#                 [3*0**2, 2*0**1, 0**0,   0],
#                 [6*0**1, 2*0**0,   0,   0],
#                 [  mid_t**3,   mid_t**2, mid_t**1, mid_t**0],
#                 [3*mid_t**2, 2*mid_t**1, mid_t**0,       0],
#                 [6*mid_t**1, 2*mid_t**0,       0,       0],
#                 [  ed**3,   ed**2, ed**1, ed**0],
#                 [3*ed**2, 2*ed**1, ed**0,    0],
#                 [6*ed**1, 2*ed**0,    0,    0]])

#     B = np.array([[start[0], start[2]],
#                 [0.0, 0.0],
#                 [0.0, 0.0],,
#                 [mid[0], mid[2]],
#                 [mid[1], mid[3]],
#                 [0.0, 0.0],,
#                 [goal[0], goal[2]],
#                 [0.0, 0.0],
#                 [0.0, 0.0]])

#     # Ax = B
#     # A.T*A*x = A.T*B
#     # x = (A.T*A)^-1*A.T*B

#     # print("A: ",A.shape)
#     # print("B: ",B.shape)

#     x = np.dot(np.dot(la.inv(np.dot(A.T,A)),A.T),B)
#     # print(x)

#     # x = 4x2

#     traj = []
#     for i in range(time_step):
#         t = i * env.samp_t
#         A_ = np.array([[  t**3,   t**2, t**1, t**0],
#                     [3*t**2, 2*t**1, t**0,    0]])   #2x4
#         B_ = np.dot(A_,x) #2x2

#         # print("B_",B_)
#         # print("ojb", B_[0,0][0])

#         traj.append([B_[0,0][0],B_[1,0][0],B_[0,1][0],B_[1,1][0]])
#         # print(np.array(traj).shape)
#     print(traj[-1])
#     return np.array(traj)

# sim_time = 5
# time_step = int(sim_time/env.samp_t)
# start = np.array([[0.0, 0.0, 0.0, 0.0]]).T
# goal = np.array([[30.0, 0.0, 20.0, 0.0]]).T

# traj = min_snap_traj(time_step, start, goal)
# print(traj.shape)