
# coding: utf-8

# In[1]:


import vrep
import time
import numpy as np
from scipy import linalg
import math
import random


# # Forward Kinematics

# In[2]:


def skew(m):
    return np.array([[0 ,-m[2], m[1]],[m[2], 0, -m[0]], [-m[1], m[0], 0]])


# In[3]:


def revolute(a,q):
    ab = skew(a)
    bottom = -1*np.dot(ab,q)
    return np.array([[a[0]],[a[1]],[a[2]],[bottom[0]],[bottom[1]],[bottom[2]]])


# In[4]:


def brack6(a):
    ws = a[:3]
    vs = a[3:]
    wb = skew(ws)
    ans = np.array([[wb[0,0], wb[0,1], wb[0,2], vs[0]],[wb[1,0], wb[1,1], wb[1,2], vs[1]],[wb[2,0], wb[2,1], wb[2,2], vs[2]], [0,0,0,0]])
    return ans


# In[5]:


def adj(a):
    R = np.array(a[:3,:3])
    p = np.array(a[:3,3])
    ret_top = np.hstack((R, np.zeros((3,3))))
    ret_bottom = np.hstack((skew(p).dot(R), R))
    ret = np.vstack((ret_top,ret_bottom))
    return ret


# In[6]:


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])


# In[7]:


def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


# In[8]:


def euclideanDistance(p1,p2):
    return math.sqrt(math.pow(p1[0]-p2[0],2)+math.pow(p1[1]-p2[1],2)+math.pow(p1[2]-p2[2],2))


# In[9]:


def forward_kinematics(theta,screw,M=np.array([[0,0,-1, -0.1940], [0,1,0,0], [1,0,0,0.6511], [0,0,0,1]])):
    
    a0 = np.array([0,0,1])
    q0 = np.array([0,0,0.1045])
    
    a1 = np.array([-1,0,0])
    q1 = np.array([-0.115,0,0.1089])
    
    a2 = np.array([-1,0,0])
    q2 = np.array([-0.115,0,0.3525])
    
    a3 = np.array([-1,0,0]) #Not bad here
    q3 = np.array([-0.115,0,0.5658])
    
    a4 = np.array([0,0,1]) #this joint also doesn't look bad
    q4 = np.array([-0.112,0,0.65])
    
    a5 = np.array([-1,0,0])  #this doesn't look bad, but hard to tell with this joint
    q5 = np.array([-0.11,0,0.6511])
    
    
#     M = np.array([[0,0,-1, -0.1940], [0,1,0,0], [1,0,0,0.6511], [0,0,0,1]])
    
    axis = [a0,a1,a2,a3,a4,a5]
    point = [q0,q1,q2,q3,q4,q5]
    
#     screw = [revolute(axis[a],point[a]) for a in range(len(axis))]
    exp = [linalg.expm(brack6(screw[s])*theta[s]) for s in range (len(screw))]
    
    ex_multiplied = np.identity(4)
    for i in exp:
        ex_multiplied = ex_multiplied.dot(i)
    T = ex_multiplied.dot(M)
    return T


# In[10]:


def finalpos(S, theta, start):
    start = np.transpose(np.array(start))
    position = start[:,:2]
    S = np.array(S)
    for i in range(2,8):
        M = np.identity(4)
        M[0,3] = start[0, i]
        M[1,3] = start[1, i]
        M[2,3] = start[2, i]
        T = forward_kinematics(theta[0:i-1],np.transpose(S[:, 0:i-1])[0], M)
        position = np.concatenate((position, T[:3, 3:4]),axis=1)

    return position


# In[11]:


def inverse_kinematics(T):
    a0 = np.array([0,0,1])
    q0 = np.array([0,0,0.1045])
    
    a1 = np.array([-1,0,0])
    q1 = np.array([-0.115,0,0.1089])
    
    a2 = np.array([-1,0,0])
    q2 = np.array([-0.115,0,0.3525])
    
    a3 = np.array([-1,0,0]) #Not bad here
    q3 = np.array([-0.115,0,0.5658])
    
    a4 = np.array([0,0,1]) #this joint also doesn't look bad
    q4 = np.array([-0.112,0,0.65])
    
    a5 = np.array([-1,0,0])  #this doesn't look bad, but hard to tell with this joint
    q5 = np.array([-0.11,0,0.6511])
    
    
    M = np.array([[0,0,-1, -0.1940], [0,1,0,0], [1,0,0,0.6511], [0,0,0,1]])
    
    axis = [a0,a1,a2,a3,a4,a5]
    point = [q0,q1,q2,q3,q4,q5]
    
    screw = [revolute(axis[a],point[a]) for a in range(len(axis))]
    
    theta = np.repeat(np.pi/8,6).reshape(6,1)
    tolerance = 0.01
    error = 20
    num_iterations = 0
    while (error > tolerance) and (num_iterations < 1000):
        exp = [linalg.expm(brack6(screw[s])*theta[s]) for s in range(len(screw))]
        Tc = np.identity(4)
        for i in exp:
            Tc = Tc.dot(i)
        Tc = Tc.dot(M)
        vb = linalg.logm(T.dot(np.linalg.inv(Tc)))
        v = np.array([vb[2,1],vb[0,2],vb[1,0],vb[0,3],vb[1,3],vb[2,3]]).reshape(6,1)
        
        jacobian = []
        jacobian.append(screw[0])
        j = np.identity(4)
        for i in range(len(exp)-1):
            j = j.dot(exp[i])
            jacobian.append(adj(j).dot(screw[i+1]))
            
        space = np.hstack((i for i in jacobian))
        
        thetadot = (np.linalg.inv(np.add(np.transpose(space).dot(space),0.1*np.eye(6)))).dot(np.transpose(space).dot(v))
        theta = np.add(theta,thetadot)
        n = np.linalg.norm(v)
        error = abs(n)
        num_iterations+=1
    print ("Iterations: ",num_iterations)
    if (num_iterations == 1000):
        return None
    return theta   


# In[14]:


# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

joint_handles = [0 for i in range(6)]
for i in range(6):
    result, joint_handles[i] = vrep.simxGetObjectHandle(clientID, 'UR3_joint'+str(i+1), vrep.simx_opmode_blocking)


result, connector = vrep.simxGetObjectHandle(clientID, 'BaxterVacuumCup', vrep.simx_opmode_blocking)
if result == -1:
    print ("not connected gripper")
    
#pick up the cuboid
T_0 = np.array([[0,1,0, 0], [0,0,1, 0.22], [1,0,0 ,0.4], [0,0,0,1]])
theta = inverse_kinematics(T_0)

if theta is None:
    print ("not found IK in finding cuboid")
else:
    vrep.simxPauseCommunication(clientID,True)
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta[i], vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID,False)
time.sleep(1)
cup = "BaxterVacuumCup_active"
vrep.simxSetIntegerSignal(clientID, cup, 1, vrep.simx_opmode_oneshot)
time.sleep(1)


theta_home = [0 for i in range(6)]


#go to conveyor 
T_1 = np.array([[0,1,0, 0.4], [0,0,1, 0], [1,0,0 ,0.35], [0,0,0,1]])
theta = inverse_kinematics(T_1)

if theta is None:
    print ("not fuoud IK in placing cuboid")
else:
    vrep.simxPauseCommunication(clientID,True)
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta[i], vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID,False)
time.sleep(1) 


#vrep.simxSetJointTargetPosition(clientID, joint_handles[3], theta[3], vrep.simx_opmode_oneshot)
#time.sleep(2)
#drop the cuboid
vrep.simxSetIntegerSignal(clientID, cup, 0, vrep.simx_opmode_oneshot)

time.sleep(2)


#return to home position
#get prepared for the second object(cup)
vrep.simxPauseCommunication(clientID,True)
for i in range(6):
    vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta_home[i], vrep.simx_opmode_oneshot)
vrep.simxPauseCommunication(clientID,False)
vrep.simxSetIntegerSignal(clientID, cup, 1, vrep.simx_opmode_oneshot)



#pick up the cup
T_2 = np.array([[0,1,0, -0.3], [0,0,1,0.24], [1,0,0,0.4], [0,0,0,1]])

theta = inverse_kinematics(T_2)
if theta is None:
    print ("IK not found in finding cup")
else:
    vrep.simxPauseCommunication(clientID,True)
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta[i], vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID,False) 

cup = "BaxterVacuumCup_active"
# Turn suction on
vrep.simxSetIntegerSignal(clientID,cup,1,vrep.simx_opmode_oneshot)

# Wait four seconds
time.sleep(4)

#go to the basin and pour the cup
theta[0] = theta[0] + np.pi/2
vrep.simxPauseCommunication(clientID,True)
for i in range(6):
    vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta[i], vrep.simx_opmode_oneshot)
vrep.simxPauseCommunication(clientID,False) 
time.sleep(1)

#pour out cup
theta[5] = theta[5] + np.pi
vrep.simxSetJointTargetPosition(clientID, joint_handles[5], theta[5], vrep.simx_opmode_oneshot)
time.sleep(1)


#put cup back to table
T = np.array([[0,1,0, -0.3], [0,0,1,0.24], [1,0,0,0.4], [0,0,0,1]])
theta = inverse_kinematics(T)
if theta is None:
    print ("No IK found in placing cup back")
else:
    vrep.simxPauseCommunication(clientID,True)
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta[i], vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID,False) 
time.sleep(1)
vrep.simxSetIntegerSignal(clientID, cup, 0, vrep.simx_opmode_oneshot)
time.sleep(1)

#return to home position
vrep.simxPauseCommunication(clientID,True)
for i in range(6):
    vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta_home[i], vrep.simx_opmode_oneshot)
vrep.simxPauseCommunication(clientID,False)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)

