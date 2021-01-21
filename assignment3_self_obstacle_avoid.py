import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 16
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [15, 0, 0]
        self.reference2 = None
        self.reference3 = None

        self.x_obs = 9
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        a_t = pedal
        beta = steering


        x_t += v_t*np.cos(psi_t)*dt
        y_t +=v_t*np.sin(psi_t)*dt
        v_t += a_t*dt - v_t/25
        psi_t += v_t*np.tan(beta)*dt/2.5
        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        tol = 1
        for k in range(0,self.horizon):
            v_start = state[3] 
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            distance = ((state[0]- self.x_obs)**2 + (state[1]- self.y_obs) **2) **0.5
            if ((distance <=tol)):
            #     # print("Distance is")
            #     # print(distance)
                cost += 1/distance*30 + (state[0]- ref[0])*(state[0]- ref[0])+(state[1]- ref[1])*(state[1]- ref[1])+(state[2]- ref[2])*(state[2]- ref[2])
            else:
                
                cost += (state[0]- ref[0])*(state[0]- ref[0])+(state[1]- ref[1])*(state[1]- ref[1])+(state[2]- ref[2])*(state[2]- ref[2])
                
            
            
           

        return cost

sim_run(options, ModelPredictiveControl)
