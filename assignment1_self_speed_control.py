import numpy as np
import sys
sys.path.insert(0, 'E:\git_wroks\MPC_gurunayk')
from sim.sim1d_self import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False # To increase the Speed of calculation on Computer
steering = 0

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 25
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        x = int(input("Enter target x co-ordinate : "))
        self.reference = [x, 0, 0]
        

    def plant_model(self, prev_state, dt, pedal, steering):
        a_t = pedal
        x_t = prev_state[0] # Only one dimensional
        v_t = prev_state[3] # m/s
        x_dot = v_t
        v_dot = a_t

        x_t_1 = x_t + v_t*dt # x_t Present state, x_t_1 is next state
        v_t_1 = v_t + a_t*dt - v_t/25.0 # v_t/25 is air friction model


        return [x_t_1, 0, 0, v_t_1] # Resturnibng Only the next state and x direction velocity,
                                    # not Resturning the y_t_1 and psi_t_1 (steering)

    def cost_function(self,u, *args):
        cost = 0
        state = args[0] # Reading the state
        ref = args[1]  # [x_t_1, 0, 0, v_t_1]0 in this array it will read the ref postion [50, 0, 0]
        costall = np.zeros([])
        pedal_list = []
        for k in range(2,self.horizon):
            v_start = state[3]  # Reading the initial velocity
            # Calculate all the states

            state = self.plant_model(state, self.dt, u[k*2], u[2*k+1])  # steering is zero. 
            #Only Pedal is given as input. Therefore the u[2*k] instaed of u[k]

            #x_current = self.plant_model(pedal_list[k])

            cost = abs(state[0] - ref[0])**2
        cost += cost
        print("Cost function is  = {cost:1.3f} and its length/size is" .format(cost=cost))
        #state = args[0]
        #cost = 0.0
        return cost

for w in range(1,2):
    sim_run(options, ModelPredictiveControl)
    w+=1
