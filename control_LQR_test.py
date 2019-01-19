import matplotlib.pyplot as plt
import numpy as np
from ctllib import dlqr
from matplotlib import animation

#Physical Variables
m = 1.0 #kg
g = 9.81 #m/s^2

#Command and control parameters
target_z = np.array([55.0, 0.0]) #meter command position
samp_t = 0.01
time_step = 1000

#initialization
time = np.linspace(0,time_step, time_step+1)
state_vector = np.zeros((len(time), 2))
state_vector[0] = np.array([-10, 0]) #initial state
inputSq = [0]
total_error = 0
first = True

#Define the system
A = np.array([[1.0, samp_t], [0.0, 1.0]])
B = np.array([0.0, samp_t*(1.0/m)])
Q = np.array([[10.0,0.0],[0.0,10.0]])
R = np.array([[1.0]])

def transition(current_state, input):
    return np.dot(A, current_state) + input*B

def LQR_regulator(state, target_z):
    K, P, eigVals = dlqr(A,np.reshape(B,(2,1)),Q,R)
    print("shapeK: ", K.shape )
    print("shape state: ", state.shape)
    u = np.dot(-K, (state - target_z))
    return u

for i in range(len(time)-1):
    error = target_z[0] - state_vector[i,0]
    if error < np.e**-4 and first:
        print("Error is ", error, " at time ", i*samp_t)
        first = False

    #Calculate the input and get next state
    u = LQR_regulator(state_vector[i], target_z)
    state_vector[i+1] = transition(state_vector[i], u)
    #memorize input
    inputSq.append(u)

print(state_vector[-1])

fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax1.plot(time*samp_t, state_vector[:,0], 'r')
ax1.plot(time*samp_t, state_vector[:,1], 'b')
ax1.set(title='LQR control', xlabel='time', ylabel='pos z, vel v' )
ax1.legend(loc = 'best')
ax2 = fig.add_subplot(2,1,2)
ax2.plot(time*samp_t, inputSq, 'g')
ax2.set(xlabel='time', ylabel='input u' )
ax2.legend(loc = 'best')
# ax1.xaxis.set(ticks=range(1,5), ticklabels=[3,100,-12,"foo"])

plt.show()

