import numpy as np
import env

class Drone():
    def __init__(self, init_state, target_state):
        #Command state 
        self.target = target_state
        #initialization
        self.state = init_state
        self.first = True
        self.last_error = np.reshape(self.target[0], (4,1)) - self.state
        self.total_error = np.zeros((4,1))
        # # Tolerence
        # self.tol = env.tolerence_is(self.state, self.target)
        #Contol parameters
        self.P = 4.5 #stable when 1.5
        self.I = 0.001 #stable when 0.001
        self.D = -400.0 #stable when -300.0

    def controller(self, i):
        #Get error
        error = np.reshape(self.target[i], (4,1)) - self.state
        self.total_error += error
        # self.first = env.tolerence_check(self.state, self.target, self.tol, self.first, i)
        #Calculate the input and get next state
        u = self.P*error + self.D*(self.last_error - error) + self.I*(self.total_error) # 4x1
        # u = np.array([[min(u[0,0], 20.0)],[min(u[2,0], 20.0)]]) # 2x1
        u = np.array([[u[0,0]],[u[2,0]]]) # 2x1
        self.state = env.transition(self.state, u)
        self.last_error = error
        return self.state