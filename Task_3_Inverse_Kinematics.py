
# coding: utf-8

# In[1]:


import vrep
import time
import numpy as np

from scipy import linalg
from random import *
import random
from scipy.linalg import logm, expm, norm, inv


def skew(s):
             
                return np.array([[0 ,-s[2] ,s[1]], [s[2], 0 ,-s[0]],[-s[1] ,s[0], 0]])
        
def skew_S(s):
                A = np.array([[0 ,-s[0][2] ,s[0][1], s[0][3]], [s[0][2], 0 ,-s[0][0], s[0][4]],[-s[0][1] ,s[0][0], 0, s[0][5]],[0,0,0,0]])
                
                return A
            
def invert(T):
                
                R=T[[0,1,2],:][:,[0,1,2]]
                
                A = np.block([[R.transpose(), (-1)*(R.transpose()).dot(T[[0,1,2],:][:,[3]])],   [0,0,0,1]    ])

                return A
            
def Adj(T):
    
                A = np.block([[T[[0,1,2],:][:,[0,1,2]], np.zeros([3,3])]  ,  [ (skew(T[[0,1,2],:][:,[3]])).dot(T[[0,1,2],:][:,[0,1,2]]),  T[[0,1,2],:][:,[0,1,2]]  ]   ])
                
                return A
            
# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')

# Get "handle" to the second joint of robot
result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for second joint')
    
# Get "handle" to the third joint of robot
result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for third joint')
    
# Get "handle" to the fourth joint of robot
result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')
    
# Get "handle" to the fifth joint of robot
result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')
    
# Get "handle" to the sixth joint of robot
result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')
    
# Get "handle" to the robot
result, robot_handle = vrep.simxGetObjectHandle(clientID, 'UR3_link1_visible', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for robot')
        
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

pret, rjoint1 = vrep.simxGetObjectPosition(clientID, joint_one_handle,-1, vrep.simx_opmode_oneshot_wait)
print('The position of the first joint is {} '.format( rjoint1))


#SECOND JOINT


# Get the current value of the second joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get second joint variable')
print('current value of second joint variable: theta = {:f}'.format(theta))

pret, rjoint2 = vrep.simxGetObjectPosition(clientID, joint_two_handle,-1, vrep.simx_opmode_oneshot_wait)
print('The position of the second joint is {} '.format( rjoint2))


# THIRD JOINT

# Get the current value of the third joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get third joint variable')
print('current value of third joint variable: theta = {:f}'.format(theta))

pret, rjoint3 = vrep.simxGetObjectPosition(clientID, joint_three_handle,-1, vrep.simx_opmode_oneshot_wait)
print('The position of the third joint is {} '.format( rjoint3))



#FOURTH JOINT

# Get the current value of the fourth joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get fourth joint variable')
print('current value of fourth joint variable: theta = {:f}'.format(theta))

pret, rjoint4 = vrep.simxGetObjectPosition(clientID, joint_four_handle,-1, vrep.simx_opmode_oneshot_wait)
print('The position of the fourth joint is {} '.format( rjoint4))


#FIFTH JOINT

# Get the current value of the fifth joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get fifth joint variable')
print('current value of fifth joint variable: theta = {:f}'.format(theta))

pret, rjoint5 = vrep.simxGetObjectPosition(clientID, joint_five_handle,-1, vrep.simx_opmode_oneshot_wait)
print('The position of the fifth joint is {} '.format( rjoint5))


#SIXTH JOINT


# Get the current value of the sixth joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sixth joint variable')
print('current value of sixth joint variable: theta = {:f}'.format(theta))


pret, rjoint6 = vrep.simxGetObjectPosition(clientID, joint_six_handle,-1, vrep.simx_opmode_oneshot_wait)
print('The position of the sixth joint is {} '.format( rjoint6))



#END - EFFECTOR

result, end_effector_handle = vrep.simxGetObjectHandle(clientID,'UR3_connection', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, end_effector_pos=vrep.simxGetObjectPosition(clientID,end_effector_handle,joint_one_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get current frame current position')


# In[2]:


theta_vec = np.array([[0.1,0.1,0.1,0.1,0.1,0.1]])


# In[3]:


q1=[0,0,0.109]
q2=[-0.12,0,0.109]
q3=[-0.12,0,0.353]
q4=[-0.027,0,0.566]
q5=[-0.11,0,0.566]
q6=[-0.11,0,0.649]
q7=[-0.192,0,0.649]

a1 = [0,0,1]
a2 = [-1,0,0]
a3 = [-1,0,0]
a4 = [-1,0,0]
a5 = [0,0,1]
a6 = [-1,0,0]

S1 = [a1,-np.dot(skew(a1),q1)]
S1 = np.reshape(S1,(1,6))
S2 = [a2,-np.dot(skew(a2),q2)]
S2 = np.reshape(S2,(1,6))
S3 = [a3,-np.dot(skew(a3),q3)]
S3 = np.reshape(S3,(1,6))
S4 = [a4,-np.dot(skew(a4),q4)]
S4 = np.reshape(S4,(1,6))
S5 = [a5,-np.dot(skew(a5),q5)]
S5 = np.reshape(S5,(1,6))
S6 = [a6,-np.dot(skew(a6),q6)]
S6 = np.reshape(S6,(1,6))

M=[[0,0,-1,q7[0]],[0,1,0,q7[1]],[1,0,0,q7[2]],[0,0,0,1]]




# In[4]:


def rand_rotation_matrix(deflection=1.0, randnums=None):
    """
    Creates a random rotation matrix.
    
    deflection: the magnitude of the rotation. For 0, no rotation; for 1, competely random
    rotation. Small deflection => small perturbation.
    randnums: 3 random numbers in the range [0, 1]. If `None`, they will be auto-generated.
    """
    # from http://www.realtimerendering.com/resources/GraphicsGems/gemsiii/rand_rotation.c
    
    if randnums is None:
        randnums = np.random.uniform(size=(3,))
        
    theta, phi, z = randnums
    
    theta = theta * 2.0*deflection*np.pi  # Rotation about the pole (Z).
    phi = phi * 2.0*np.pi  # For direction of pole deflection.
    z = z * 2.0*deflection  # For magnitude of pole deflection.
    
    # Compute a vector V used for distributing points over the sphere
    # via the reflection I - V Transpose(V).  This formulation of V
    # will guarantee that if x[1] and x[2] are uniformly distributed,
    # the reflected points will be uniform on the sphere.  Note that V
    # has length sqrt(2) to eliminate the 2 in the Householder matrix.
    
    r = np.sqrt(z)
    Vx, Vy, Vz = V = (
        np.sin(phi) * r,
        np.cos(phi) * r,
        np.sqrt(2.0 - z)
        )
    
    st = np.sin(theta)
    ct = np.cos(theta)
    
    R = np.array(((ct, st, 0), (-st, ct, 0), (0, 0, 1)))
    
    # Construct the rotation matrix  ( V Transpose(V) - I ) R.
    
    Rot = (np.outer(V, V) - np.eye(3)).dot(R)
    return Rot


# In[5]:


#Poses that are reachable

#array([[-0.30685571, -0.30558095, -0.90136555,  0.25878615],
#       [ 0.3467645 , -0.9178555 ,  0.19312086,  0.32381135],
#       [-0.88633738, -0.25330133,  0.38761382,  0.32689787],
#       [ 0.        ,  0.        ,  0.        ,  1.        ]])



#T_1in0=np.array([[ 0.58341152,  0.78387923,  0.21251908,  0.21779692],
#       [ 0.18695602, -0.38425629,  0.90409875,  0.34849514],
#       [ 0.79036602, -0.4877299 , -0.37073049,  0.27053918],
#       [ 0.        ,  0.        ,  0.        ,  1.        ]])

#T_1in0 = np.array([[-0.03957477, -0.96134411,  0.27249099,  0.25685692],
 #      [ 0.63132818,  0.18732024,  0.7525529 ,  0.25710661],
  #     [-0.77450537,  0.20181335,  0.59951047,  0.3646851 ],
   #    [ 0.        ,  0.        ,  0.        ,  1.        ]])


#Poses not reachable
#array([[-0.54594909,  0.09068134,  0.83289644,  0.3071057 ],
#       [ 0.82618571, -0.10681461,  0.55317973,  0.37351298],
#       [ 0.13912859,  0.99013511, -0.01660429,  0.3699312 ],
#       [ 0.        ,  0.        ,  0.        ,  1.        ]])



    
Rot=rand_rotation_matrix(1,None)


p1 = random.uniform(0.2,0.3)
p2 = random.uniform(0.25,0.35)
p3 = random.uniform(0.2,0.4)

p_T = np.array([[p1],[p2],[p3]])

print(p_T)
    
#T_1in0 = np.block([[Rot,p_T], [0,0,0,1]]) 


T_1in0=np.array([[ 0.58341152,  0.78387923,  0.21251908,  0.21779692],
       [ 0.18695602, -0.38425629,  0.90409875,  0.34849514],
       [ 0.79036602, -0.4877299 , -0.37073049,  0.27053918],
      [ 0.        ,  0.        ,  0.        ,  1.        ]])




frame_target_pos = [T_1in0[0][3],T_1in0[1][3],T_1in0[2][3]]
print('Target position:', frame_target_pos)

result, frame_handle = vrep.simxGetObjectHandle(clientID,'ReferenceFrame', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, current_frame_pos=vrep.simxGetObjectPosition(clientID,frame_handle,robot_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get current frame current position')
    
print('Position before change:', current_frame_pos)   

result = vrep.simxSetObjectPosition(clientID,frame_handle,robot_handle,frame_target_pos,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    
    
result, current_frame_pos=vrep.simxGetObjectPosition(clientID,frame_handle,robot_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get current frame current position')
print('Final position:', current_frame_pos) 


beta=np.arcsin(T_1in0[0][2])
alfa=np.arccos(np.divide(T_1in0[2][2],np.cos(beta)))
gamma = np.arccos(np.divide(T_1in0[0][0],np.cos(beta)))

Euler_angles = ([(-1)*alfa,beta,(-1)*gamma])

#sy = np.sqrt(np.add(T_1in0[0][0]*T_1in0[0][0],T_1in0[1][0]*T_1in0[1][0]))
#phi = np.arctan2(T_1in0[1][0], T_1in0[0][0])
#tet = np.arctan2(T_1in0[2][0], sy)
#chi = np.arctan2(T_1in0[2][1], T_1in0[2][2])

#Euler_angles = ([chi,tet,phi])

print('Final orientation:', Euler_angles)

result =vrep.simxSetObjectOrientation(clientID,frame_handle,robot_handle, Euler_angles, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame final orientation')


# In[6]:



e1 = expm(skew_S(S1)*theta_vec[0][0])
e2 = expm(skew_S(S2)*theta_vec[0][1])
e3 = expm(skew_S(S3)*theta_vec[0][2])
e4 = expm(skew_S(S4)*theta_vec[0][3])
e5 = expm(skew_S(S5)*theta_vec[0][4])
e6 = expm(skew_S(S6)*theta_vec[0][5])


T = (e1).dot(e2).dot(e3).dot(e4).dot(e5).dot(e6).dot(M)
print(T)

T_inv=invert(T)
print(T_inv)


# In[7]:


skew_B = logm((T_inv).dot(T_1in0))
B = np.block([[(-1)*skew_B[1,2]], [skew_B[0,2]] ,[(-1)*skew_B[0,1] ], [skew_B[[0,1,2],:][:,[3]] ]])


# In[8]:


i=0
stop=0

while norm(B)>0.01 and stop == 0: 
    e1 = expm(skew_S(S1)*theta_vec[0][0])
    e2 = expm(skew_S(S2)*theta_vec[0][1])
    e3 = expm(skew_S(S3)*theta_vec[0][2])
    e4 = expm(skew_S(S4)*theta_vec[0][3])
    e5 = expm(skew_S(S5)*theta_vec[0][4])
    e6 = expm(skew_S(S6)*theta_vec[0][5])
    
    
    J1 = np.array(S1)
    J2 = np.array(Adj(e1)).dot(S2.transpose())
    J3 = np.array(Adj((e1).dot(e2))).dot(S3.transpose())
    J4 = np.array(Adj((e1).dot(e2).dot(e3))).dot(S4.transpose())
    J5 = np.array(Adj((e1).dot(e2).dot(e3).dot(e4))).dot(S5.transpose())
    J6 = np.array(Adj((e1).dot(e2).dot(e3).dot(e4).dot(e5))).dot(S6.transpose())
   
    T = (e1).dot(e2).dot(e3).dot(e4).dot(e5).dot(e6).dot(M)
    print(T)


    T_inv=invert(T)
    
    
    
    J = np.concatenate((J1,J2.T,J3.T,J4.T,J5.T,J6.T))
    J=J.T
    
    amplV = logm((T_1in0).dot(T_inv))
    
    V= np.block([[(-1)*amplV[1,2]], [amplV[0,2]] ,[(-1)*amplV[0,1] ], [amplV[[0,1,2],:][:,[3]]] ])
    
    thetadot = np.array((inv(J)).dot(V))
    theta_vec = np.add(theta_vec,0.02*thetadot.transpose())
    
    skew_B = logm((T_inv).dot(T_1in0))
    B = np.block([[(-1)*skew_B[1,2]], [skew_B[0,2]] ,[(-1)*skew_B[0,1] ], [skew_B[[0,1,2],:][:,[3]] ]])
    print(norm(B))
        # Set the desired value for each joint variable
    result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of first joint variable: theta = {:f}'.format(theta))

    vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta_vec[0][0], vrep.simx_opmode_oneshot)
    #time.sleep(2)

    result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of first joint variable: theta = {:f}'.format(theta))



    result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of second joint variable: theta = {:f}'.format(theta))

    vrep.simxSetJointTargetPosition(clientID, joint_two_handle, theta_vec[0][1], vrep.simx_opmode_oneshot)
    #time.sleep(2)

    result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of second joint variable: theta = {:f}'.format(theta))



    result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of third joint variable: theta = {:f}'.format(theta))

    vrep.simxSetJointTargetPosition(clientID, joint_three_handle, theta_vec[0][2], vrep.simx_opmode_oneshot)
    #time.sleep(2)

    result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of third joint variable: theta = {:f}'.format(theta))


    result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of fourth joint variable: theta = {:f}'.format(theta))

    vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta_vec[0][3], vrep.simx_opmode_oneshot)
    #time.sleep(2)

    result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of fourth joint variable: theta = {:f}'.format(theta))


    result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of fifth joint variable: theta = {:f}'.format(theta))

    vrep.simxSetJointTargetPosition(clientID, joint_five_handle, theta_vec[0][4], vrep.simx_opmode_oneshot)
    #time.sleep(2)

    result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of fifth joint variable: theta = {:f}'.format(theta))


    result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of sixth joint variable: theta = {:f}'.format(theta))


    vrep.simxSetJointTargetPosition(clientID, joint_six_handle, theta_vec[0][5], vrep.simx_opmode_oneshot)
    #time.sleep(2)


    result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get first joint variable')
    print('current value of sixth joint variable: theta = {:f}'.format(theta))
    
    result, Euler_angles_end_effector = vrep.simxGetObjectOrientation(clientID,end_effector_handle,robot_handle,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get end-effector orientation')
    print('Euler angles end effector', Euler_angles_end_effector)
    i+=1
    
    if i == 300:

        stop = 1
        print('None')


# In[ ]:


if stop == 0:
    print('Final set of thetas:', theta_vec)
else: 
    print('None')


# In[ ]:


beta=np.arcsin(T_1in0[0][2])
alfa=np.arccos(np.divide(T_1in0[2][2],np.cos(beta)))
gamma = np.arccos(np.divide(T_1in0[0][0],np.cos(beta)))

Euler_angles = ([beta,alfa,gamma])

print(Euler_angles)


# In[ ]:


result, end_effector_pos=vrep.simxGetObjectPosition(clientID,end_effector_handle,robot_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get current frame current position')
print (end_effector_pos)

