import numpy as np
import env
from ctllib import dlqr

class Drone():
    def __init__(self, init_state, target_state):
        #Command state 
        self.target = target_state
        #initialization
        self.state = init_state
        self.first = True
        # #Tolerence
        # self.tol = env.tolerence_is(self.state, self.target)
        #LQR Cost Funcion
        self.Q = np.array([[1000.0,0.0,0.0,0.0],    #Gain for x_pos
                            [0.0,1.0,0.0,0.0],      #Gain for x_vel
                            [0.0,0.0,1000.0,0.0],   #Gain for y_pos
                            [0.0,0.0,0.0,1.0]])     #Gain for y_vel
                            # 4x4
        self.R = np.array([[0.1, 0.0], [0.0, 0.1]]) # 2x2

    def LQR_regulator(self, state, target):
        K, P, eigVals = dlqr(env.A, env.B, self.Q, self.R)
        u = np.dot(-K, (state - target))  #2x1
        return u

    def controller(self, i):
        # self.first = env.tolerence_check(self.state, self.target, self.tol, self.first, i)
        #Calculate the input and get next state
        u = self.LQR_regulator(self.state, np.reshape(self.target[i], (4,1)))
        self.state = env.transition(self.state, u)
        return self.state