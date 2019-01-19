import matplotlib.pyplot as plt
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
import control_LQR
import control_PID
import env
import traj_generator as tj
import numpy as np

class Animator(): 
    def __init__(self, sim_time, init_state, goal_state, traj_type, ctl_type):
        #Define the time step
        self.frames = int(sim_time/env.samp_t)
        #Set the initial state and the target state
        self.init_state = init_state
        self.goal_state = goal_state
        #Select the Trajectory generator
        if traj_type == "sin":
            self.traj_plan = tj.sin_traj(self.frames, init_state) #meter, The sequence of [cmd_pos_x, cmd_vel_x, cmd_pos_y, cmd_vel_y] 4xtimestep
        elif traj_type == "line":
            self.traj_plan = tj.line_traj(self.frames, init_state, goal_state)
        elif traj_type == "poly":
            self.traj_plan = tj.min_snap_traj(self.frames, init_state, goal_state)
        else:
            self.traj_plan = np.array([[goal_state[0], 0.0, goal_state[2], 0.0]]*self.frames)
        #Select the control method
        if ctl_type == "LQR":
            self.dr = control_LQR.Drone(self.init_state, self.traj_plan)
        elif ctl_type == "PID":
            self.dr = control_PID.Drone(self.init_state, self.traj_plan)
        #Data stroage
        self.time = [-1]
        self.x_vector = [self.dr.state[0]]
        self.y_vector = [self.dr.state[2]]
        #Animation setting
        self.fig = plt.figure(figsize=(20.0,10.0))
        self.ax1 = self.fig.add_subplot(121, xlim=(-40, 40), ylim=(-40, 40))
        self.ax2 = self.fig.add_subplot(122, xlim=(0, self.frames), ylim=(-40, 40))
        self.drone, = self.ax1.plot([],[], "ro", ms="12")
        self.planGraph, = self.ax1.plot([],[], "g")
        self.x_posGraph, = self.ax2.plot([],[], "r")
        self.y_posGraph, = self.ax2.plot([],[], "b")
    # initialization function: plot the background of each frame
    def init(self):
        self.drone.set_data([], [])
        self.planGraph.set_data([], [])
        self.x_posGraph.set_data([],[])
        self.y_posGraph.set_data([],[])
        return self.drone, self.planGraph, self.x_posGraph, self.y_posGraph,
    # animation function.  This is called sequentially
    def animate(self,i):
        state = self.dr.controller(i)
        self.drone.set_data([state[0]], [state[2]])
        self.planGraph.set_data(self.traj_plan[i,0],self.traj_plan[i,2])
        self.planGraph.set_data(self.traj_plan[:i,0], self.traj_plan[:i,2])
        self.time.append(i)
        self.x_vector.append(state[0])
        self.y_vector.append(state[2])
        self.x_posGraph.set_data(self.time, self.x_vector)
        self.y_posGraph.set_data(self.time, self.y_vector)
        return self.drone, self.planGraph, self.x_posGraph, self.y_posGraph,

#Set the start and goal and duration time
time = 10.0
init_state = np.array([[0.0, 0.0, 0.0, 0.0]]).T #meter [init_pos_x, init_vel_x, init_pos_y, init_vel_y] 4x1
goal_state = np.array([[30.0, 0.0, 20.0, 0.0]]).T #meter [init_pos_x, init_vel_x, init_pos_y, init_vel_y] 4x1

# call the animator.  blit=True means only re-draw the parts that have changed.
ani = Animator(time, init_state, goal_state, "sin", "LQR")
anim = animation.FuncAnimation(ani.fig, ani.animate, init_func=ani.init,
                            frames=ani.frames, interval=10, blit=True, repeat=False)
plt.show()