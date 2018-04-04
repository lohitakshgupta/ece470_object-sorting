
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
             
                return [[0 ,-s[2] ,s[1]], [s[2], 0 ,-s[0]],[-s[1] ,s[0], 0]]
        
def skew_S(s):
                A = np.array([[0 ,-s[0][2] ,s[0][1], s[0][3]], [s[0][2], 0 ,-s[0][0], s[0][4]],[-s[0][1] ,s[0][0], 0, s[0][5]],[0,0,0,0]])
                
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
    
    
    
# SPHERES IN THE ROBOT

result, sphere_joint1_handle = vrep.simxGetObjectHandle(clientID,'Dummy', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_joint2_handle = vrep.simxGetObjectHandle(clientID,'Dummy0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_joint3_handle = vrep.simxGetObjectHandle(clientID,'Dummy1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_extra1_handle = vrep.simxGetObjectHandle(clientID,'Dummy2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_joint4_handle = vrep.simxGetObjectHandle(clientID,'Dummy3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_joint5_handle = vrep.simxGetObjectHandle(clientID,'Dummy4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')
    
result, sphere_joint6_handle = vrep.simxGetObjectHandle(clientID,'Dummy5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_end_effector_handle = vrep.simxGetObjectHandle(clientID,'Dummy6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')
    

# OBSTACLES

result, sphere_obs1_handle = vrep.simxGetObjectHandle(clientID,'Dummy7', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

result, sphere_obs2_handle = vrep.simxGetObjectHandle(clientID,'Dummy8', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for frame')

        
# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

#FIRST JOINT

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


#Initial position 

q1=[0,0,0.109]
q2=[-0.12,0,0.109]
q3=[-0.12,0,0.353]
q4=[-0.027,0,0.566]
q5=[-0.11,0,0.566]
q6=[-0.11,0,0.649]
q7=[-0.192,0,0.649]  #end effector

extra1 = [-0.027,0,0.353]


result, obs1_pos=vrep.simxGetObjectPosition(clientID,sphere_obs1_handle,robot_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get current frame current position')
    
result, obs2_pos=vrep.simxGetObjectPosition(clientID,sphere_obs2_handle,robot_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get current frame current position')
    
p8 = obs1_pos
p9 = obs2_pos


# In[3]:


#Spheres

result = vrep.simxSetObjectPosition(clientID,sphere_joint1_handle,robot_handle,q1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    
result = vrep.simxSetObjectPosition(clientID,sphere_joint2_handle,robot_handle,q2,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')

result = vrep.simxSetObjectPosition(clientID,sphere_joint3_handle,robot_handle,q3,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    
result = vrep.simxSetObjectPosition(clientID,sphere_extra1_handle,robot_handle,extra1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    
result = vrep.simxSetObjectPosition(clientID,sphere_joint4_handle,robot_handle,q4,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    
result = vrep.simxSetObjectPosition(clientID,sphere_joint5_handle,robot_handle,q5,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    
result = vrep.simxSetObjectPosition(clientID,sphere_joint6_handle,robot_handle,q6,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')

result = vrep.simxSetObjectPosition(clientID,sphere_end_effector_handle,robot_handle,q7,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')


# In[4]:


#Thetas for self collision

theta_vec = np.array([[0.1,0.1,3.3,0.1,0.1,0.1]])
#theta_vec = np.array([[0.1,1.8,-3.3,0.1,0.1,0.1]])


# Thetas for collision with object
#theta_vec = np.array([[0.1,1.8,0.1,0.1,0.1,0.1]])
#theta_vec = np.array([[1.8,-1.3,0.1,0.1,0.1,0.1]])


#No collision
#theta_vec = np.array([[0.1,0.1,0.1,0.1,0.1,0.1]])


# In[5]:


#FORWARD KINEMATICS

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


e1 = expm(skew_S(S1)*theta_vec[0][0])
e2 = expm(skew_S(S2)*theta_vec[0][1])
e3 = expm(skew_S(S3)*theta_vec[0][2])
e4 = expm(skew_S(S4)*theta_vec[0][3])
e5 = expm(skew_S(S5)*theta_vec[0][4])
e6 = expm(skew_S(S6)*theta_vec[0][5])


T = (e1).dot(e2).dot(e3).dot(e4).dot(e5).dot(e6).dot(M)
print(T)

final_pj2=expm(skew_S(S1)*theta_vec[0][0]).dot(np.concatenate([q2,[1]]))
final_pj3=expm(skew_S(S1)*theta_vec[0][0]).dot(expm(skew_S(S2)*theta_vec[0][1])).dot(np.concatenate([q3,[1]]))
final_pextra1=expm(skew_S(S1)*theta_vec[0][0]).dot(expm(skew_S(S2)*theta_vec[0][1])).dot(np.concatenate([extra1,[1]]))
final_pj4=expm(skew_S(S1)*theta_vec[0][0]).dot(expm(skew_S(S2)*theta_vec[0][1])).dot(expm(skew_S(S3)*theta_vec[0][2])).dot(np.concatenate([q4,[1]]))
final_pj5=expm(skew_S(S1)*theta_vec[0][0]).dot(expm(skew_S(S2)*theta_vec[0][1])).dot(expm(skew_S(S3)*theta_vec[0][2])).dot(expm(skew_S(S4)*theta_vec[0][3])).dot(np.concatenate([q5,[1]]))
final_pj6=expm(skew_S(S1)*theta_vec[0][0]).dot(expm(skew_S(S2)*theta_vec[0][1])).dot(expm(skew_S(S3)*theta_vec[0][2])).dot(expm(skew_S(S4)*theta_vec[0][3])).dot(expm(skew_S(S5)*theta_vec[0][4])).dot(np.concatenate([q6,[1]]))
final_pj7=expm(skew_S(S1)*theta_vec[0][0]).dot(expm(skew_S(S2)*theta_vec[0][1])).dot(expm(skew_S(S3)*theta_vec[0][2])).dot(expm(skew_S(S4)*theta_vec[0][3])).dot(expm(skew_S(S5)*theta_vec[0][4])).dot(expm(skew_S(S6)*theta_vec[0][5])).dot(np.concatenate([q7,[1]]))

p1 = q1;
p2 = final_pj2[0:3]
p3 = final_pj3[0:3]
pextra1 = final_pextra1[0:3]
p4 = final_pj4[0:3]
p5 = final_pj5[0:3]
p6 = final_pj6[0:3]
p7 = final_pj7[0:3]

sphere_p = np.concatenate(([p1],[p2],[p3],[p4],[p5],[p6],[p7],[p8],[p9]))
sphere_p = sphere_p.T


# In[6]:


sphere_p


# In[7]:


frame_target_pos = [T[0][3],T[1][3],T[2][3]]
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


# In[8]:


# Set the desired value for each joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta_vec[0][0], vrep.simx_opmode_oneshot)
#time.sleep(2)


result = vrep.simxSetObjectPosition(clientID,sphere_joint1_handle,robot_handle,p1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')
    

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

result = vrep.simxSetObjectPosition(clientID,sphere_joint2_handle,robot_handle,p2,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')

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

result = vrep.simxSetObjectPosition(clientID,sphere_joint3_handle,robot_handle,p3,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')


result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of third joint variable: theta = {:f}'.format(theta))


result = vrep.simxSetObjectPosition(clientID,sphere_extra1_handle,robot_handle,pextra1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')


result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of fourth joint variable: theta = {:f}'.format(theta))


vrep.simxSetJointTargetPosition(clientID, joint_four_handle, theta_vec[0][3], vrep.simx_opmode_oneshot)
#time.sleep(2)

result = vrep.simxSetObjectPosition(clientID,sphere_joint4_handle,robot_handle,p4,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')

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

result = vrep.simxSetObjectPosition(clientID,sphere_joint5_handle,robot_handle,p5,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')

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

result = vrep.simxSetObjectPosition(clientID,sphere_joint6_handle,robot_handle,p6,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')


result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of sixth joint variable: theta = {:f}'.format(theta))



result = vrep.simxSetObjectPosition(clientID,sphere_end_effector_handle,robot_handle,p7,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame new position')


# In[9]:


def IsTwoBallCollision(p1,p2,r1,r2):
    collision = 0
    diff = np.array([0.0,0.0,0.0])
    diff[0] = p1[0] - p2[0]
    diff[1] = p1[1] - p2[1]
    diff[2] = p1[2] - p2[2]
    if np.linalg.norm(diff) < (r1 + r2):
        collision = 1
    return collision



# In[10]:


sphere_r = (0.05,0.05,0.025,0.025,0.0125,0.0125,0.0125,0.5,0.25)


# In[11]:


#def CollisionCheck(sphere_p,sphere_r):
def CollisionCheck():
    collision = 0
    for i in range(0,9):
        for j in range(0,9):
            if IsTwoBallCollision(sphere_p[[0,1,2],i],sphere_p[[0,1,2],j],sphere_r[i],sphere_r[j]) == 1 and i != j:
                collision = 1
               
                break
                
    return collision


# In[12]:


collision = CollisionCheck()
print(collision)

