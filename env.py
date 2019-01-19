import numpy as np

#Physical Variables
m = 1.0 #kg drone weight
g = 9.81 #m/s^2 gravity accel.
samp_t = 0.01 #sec
#Define the system
A = np.array([[1.0, samp_t, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                    [ 0.0, 0.0, 1.0, samp_t], [0.0, 0.0, 0.0, 1.0]]) # State transition Matrix 4x4
B = np.array([[0.0, 0.0], [samp_t*(1.0/m), 0.0], [0.0, 0.0], [0.0, samp_t*(1.0/m)]]) # input 4x2
F = np.array([[0.0], [0.0], [0.0], [-g]]) # Environment force 4x1

def transition(current_state, input):
    return np.dot(A, current_state) + np.dot(B, input) + F*samp_t

#Tolerence
def tolerence_is(state, target):
    dist = state.T - target
    return np.e**-4*np.dot(dist.T, dist)

def tolerence_check(state, target, tol, first, i):
    #Get error
    error = target - state
    error = np.dot(error.T, error)/4.0
    if error[0,0] < tol[0,0] and first and i%100==0:
        print("Error is ", error[0,0], " at time ", i*samp_t)
        return False
    else:
        return True
