
import numpy as np
import matplotlib.pyplot as plt
from FBL_Controller import Controller
from dynamics import dynamics

#Simulation#
Tstop = 100
Ts = 0.1
N = int(Tstop/Ts)

#Initials
State = np.zeros(shape=(12,1))
T, T4 = 1, 0
StateStored = []
InputStored = []
U = [1,1,1,0,0,0]

#Make desired trajectory
DesiredState = np.zeros(shape=(12,1))
DesiredState[0,0] = 1 #desired trajectory along z coordinate
DesiredState[2,0] = 0 #desired trajectory along x coordinate
DesiredState[4,0] = 0 #desired trajectory along y coordinate


#Constants
K = 0.00000298
m = 0.65
g = 9.81
l = 0.23
b = 0.0000313
d = 0.00000075
kftz = 0.001
kftx = 0.00075
kfty = 0.00075
kfrz = 0.001
kfrx = 0.00075
kfry = 0.00075
J_x = 0.0075
J_y = 0.0075
J_z = 0.013
J_r = 0.00005
Constants = [K, m, g, l, b, kftz, kftx, kfty, kfrz, kfrx, kfry, J_x, J_y, J_z, J_r]

#Controllers parameters
#z 
k1 = 8.1
k2 = 15.93
#x 
k3 = 1.84
k4 = 16.49
#y 
k5 = 0.63
k6 = 10.04
#psi 
k7 = 493
k8 = 2230
#phi 
k9 = 5600
k10 = 7090
#theta 
k11 = 5100
k12 = 7590

Parameters = [k1,k2,k3,k4,k5,k6,k7,k8,k9,k10,k11,k12]

#main cycle
for i in range(N):

    #wait for drone to fly up 1m
    if i*Ts == 10:
        DesiredState[2,0] = 0.5 #desired trajectory is step function with modul of 0.5 along x coordinate
        DesiredState[4,0] = 0.5 #desired trajectory is step function with modul of 1 along y coordinate
    
    # make circular trajectory
    DesiredState[2,0] = np.sin(i*0.01) 
    DesiredState[4,0] = np.cos(i*0.01)

    #Get control signal for current step
    U = Controller(State, DesiredState, Parameters, Constants, Ts, T4, T)

    #adjust desired inputs for x and y since they cannot be directly calculated
    DesiredState[8] = np.arctan((U[0]*np.cos(DesiredState[10])+U[1]*np.sin(DesiredState[10]))/T)
    DesiredState[6] = np.arctan((U[0]*np.sin(DesiredState[10])-U[1]*np.cos(DesiredState[10]))/T )
    
    #recalculate with new trajectories
    U = np.array(Controller(State, DesiredState, Parameters, Constants, Ts, T4, T))

    #pass input and previous state through the model
    State, T, T4 = dynamics(Ts, State, U, 0)


    #store state and input for later plotting
    StateStored.append(State)
    InputStored.append(U)


#plotting the result
fig, axes = plt.subplots(nrows=2, ncols=3)
t = np.arange(0,Tstop,Ts)

plotindex=0
#indexes
# 0 = z
# 2 = x
# 4 = y
# 6 = phi
# 8 = theta
# 10 = psi


axes[0,0].plot(t, np.array(StateStored)[:,plotindex,0], color='red', linewidth=1, markersize=1) 
axes[0,0].set_title('z(t)')
axes[0,0].grid()
axes[0,0].tick_params(axis='both', which='major', pad=15)

plotindex=2
axes[0,1].plot(t, np.array(StateStored)[:,plotindex,0], color='red', linewidth=1, markersize=1) 
axes[0,1].set_title('x(t)')
axes[0,1].grid()
axes[0,1].tick_params(axis='both', which='major', pad=15)

plotindex=4
axes[0,2].plot(t, np.array(StateStored)[:,plotindex,0], color='red', linewidth=1, markersize=1) 
axes[0,2].set_title('y(t)')
axes[0,2].grid()
axes[0,2].tick_params(axis='both', which='major', pad=15)

plotindex=6
axes[1,0].plot(t, np.array(StateStored)[:,plotindex,0], color='red', linewidth=1, markersize=1) 
axes[1,0].set_title('phi(t)')
axes[1,0].grid()
axes[1,0].tick_params(axis='both', which='major', pad=15)

plotindex=8
axes[1,1].plot(t, np.array(StateStored)[:,plotindex,0], color='red', linewidth=1, markersize=1) 
axes[1,1].set_title('theta(t)')
axes[1,1].grid()
axes[1,1].tick_params(axis='both', which='major', pad=15)

plotindex=10
axes[1,2].plot(t, np.array(StateStored)[:,plotindex,0], color='red', linewidth=1, markersize=1) 
axes[1,2].set_title('psi(t)')
axes[1,2].grid()
axes[1,2].tick_params(axis='both', which='major', pad=15)

plt.tight_layout()
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(np.array(StateStored)[:,2,0] ,np.array(StateStored)[:,4,0] ,np.array(StateStored)[:,0,0], 'blue')
plt.show()
