import numpy as np
from sim.sim2d_self import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3.14/2]
        self.reference3 = [0, 10, 0*3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        a_t = pedal
        beta = steering

        x_t_1 = x_t + v_t*np.cos(psi_t)*self.dt
        y_t_1 = y_t + v_t*np.sin(psi_t)*self.dt
        v_t_1 = v_t + a_t * self.dt - v_t/25
        psi_t_1 = psi_t + (v_t * np.tan(beta)*self.dt)/2.5

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        for k in range(0,self.horizon):
            v_start = state[3] 
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            cost += (state[0]- ref[0])*(state[0]- ref[0])+(state[1]- ref[1])*(state[1]- ref[1])
            cost +=(state[2]- ref[2])*(state[2]- ref[2])
        
        return cost

sim_run(options, ModelPredictiveControl)
