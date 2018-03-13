
# coding: utf-8

# In[1]:


import vrep
import time
import numpy as np

from scipy import linalg


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


#print('Insert Thetas:')
theta_input = np.array([0.1,0.6,0.7,0.9,0.5,0.6])
#theta = [0,0,0,0,0,0]
#theta[0] = input("Theta_0:")
#theta[1] = input("Theta_1:")
#theta[2] = input("Theta_2:")
#theta[3] = input("Theta_3:")
#theta[4] = input("Theta_4:")
#theta[5] = input("Theta_5:")


# In[3]:



# Forward Kinematics

#q1_ab = rjoint1
#q2_ab = rjoint2
#q3_ab = rjoint3
#q4_ab = rjoint4
#q5_ab = rjoint5
#q6_ab = rjoint6

#q1 = [0,0,rjoint1[2]]

#q2 = np.add(np.subtract(q2_ab, q1_ab),q1)
#q3 = np.add(np.subtract(q3_ab, q1_ab),q1)
#q4 = np.add(np.subtract(q4_ab, q1_ab),q1)
#q5 = np.add(np.subtract(q5_ab, q1_ab),q1)
#q6 = np.add(np.subtract(q6_ab, q1_ab),q1)

#q7=np.add(np.subtract(end_effector_pos,q1_ab),q1)

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


e1 = linalg.expm(skew_S(S1)*theta_input[0])
e2 = linalg.expm(skew_S(S2)*theta_input[1])
e3 = linalg.expm(skew_S(S3)*theta_input[2])
e4 = linalg.expm(skew_S(S4)*theta_input[3])
e5 = linalg.expm(skew_S(S5)*theta_input[4])
e6 = linalg.expm(skew_S(S6)*theta_input[5])


T = (e1).dot(e2).dot(e3).dot(e4).dot(e5).dot(e6).dot(M)
print(T)


# In[4]:



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


# In[5]:


# Set the desired value for each joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

vrep.simxSetJointTargetPosition(clientID, joint_one_handle, theta + 0.1, vrep.simx_opmode_oneshot)
time.sleep(2)

result, theta = vrep.simxGetJointPosition(clientID, joint_one_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))



result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of second joint variable: theta = {:f}'.format(theta))

vrep.simxSetJointTargetPosition(clientID, joint_two_handle, 0.6 + theta, vrep.simx_opmode_oneshot)
time.sleep(2)

result, theta = vrep.simxGetJointPosition(clientID, joint_two_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of second joint variable: theta = {:f}'.format(theta))



result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of third joint variable: theta = {:f}'.format(theta))

vrep.simxSetJointTargetPosition(clientID, joint_three_handle, 0.7 + theta, vrep.simx_opmode_oneshot)
time.sleep(2)

result, theta = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of third joint variable: theta = {:f}'.format(theta))


result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of fourth joint variable: theta = {:f}'.format(theta))

vrep.simxSetJointTargetPosition(clientID, joint_four_handle, 0.9 + theta, vrep.simx_opmode_oneshot)
time.sleep(2)

result, theta = vrep.simxGetJointPosition(clientID, joint_four_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of fourth joint variable: theta = {:f}'.format(theta))


result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of fifth joint variable: theta = {:f}'.format(theta))

vrep.simxSetJointTargetPosition(clientID, joint_five_handle, 0.5 + theta, vrep.simx_opmode_oneshot)
time.sleep(2)

result, theta = vrep.simxGetJointPosition(clientID, joint_five_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of fifth joint variable: theta = {:f}'.format(theta))


result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of sixth joint variable: theta = {:f}'.format(theta))


vrep.simxSetJointTargetPosition(clientID, joint_six_handle, 0.6 + theta, vrep.simx_opmode_oneshot)
time.sleep(2)


result, theta = vrep.simxGetJointPosition(clientID, joint_six_handle, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of sixth joint variable: theta = {:f}'.format(theta))


# In[6]:


result, Euler_angles = vrep.simxGetObjectOrientation(clientID,end_effector_handle,robot_handle,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get end-effector orientation')
    
print('Final orientation:', Euler_angles)

result =vrep.simxSetObjectOrientation(clientID,frame_handle,robot_handle, Euler_angles, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set frame final orientation')


# In[7]:


# Stop simulation
#vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
#vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
#vrep.simxFinish(clientID)

