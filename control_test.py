import matplotlib.pyplot as plt
import numpy as np

#Physical Variables
m = 1.0 #kg
g = 9.81 #m/s^2

#Command and control parameters
target_z = 20.0 #meter command position
P = 1.5
I = 0.001
D = -300.0
samp_t = 0.01
time_step = 5000

#initialization
time = np.linspace(0,time_step, time_step+1)
state_vector = np.zeros((len(time), 2))
state_vector[0] = np.array([-10.0, 0.0]) #initial state
inputSq = [0]
last_error = target_z - state_vector[0,0]
total_error = 0
first = True


def transition(current_state, input):
    A = np.array([[1.0, samp_t], [0.0, 1.0]])
    B = np.array([0, 1.0/m])*input*samp_t
    return np.dot(A, current_state) + B #+ np.array([0, -g])*samp_t

for i in range(len(time)-1):
    #Get error
    error = target_z - state_vector[i,0]
    if error < np.e**-4 and first:
        print("Error is ", error, " at time ", i*samp_t)
        first = False
    total_error += error
    #Calculate the input and get next state
    u = P*error + D*(last_error - error) + I*(total_error)
    u = min(u, 20)
    state_vector[i+1] = transition(state_vector[i], u)
    last_error = error
    #memorize input
    inputSq.append(u)

print(state_vector[-1])

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.plot(time, state_vector[:,0], 'r')
ax1.plot(time, state_vector[:,1], 'b')
ax1.plot(time, inputSq, 'g')
plt.show()

