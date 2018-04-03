def inCollision(clientID, robo):
    # Get "handle" to the UR3 to UR3#0 collision object
    result, collision_Handle_1 = vrep.simxGetCollisionHandle(clientID, 'Collision#1', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for UR3 to UR3#0 collision object')
    
    # Get "handle" to the UR3 to MicoHand#0 collision object
result, collision_Handle_2 = vrep.simxGetCollisionHandle(clientID, 'Collision#2', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for UR3 to MicoHand#0 collision object')
    
    # Get "handle" to the MicoHand to UR3#0 collision object
result, collision_Handle_3 = vrep.simxGetCollisionHandle(clientID, 'Collision#3', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for MicoHand to UR3#0 collision object')
    
    # Get "handle" to the MicoHand to MicoHand#0 collision object
result, collision_Handle_4 = vrep.simxGetCollisionHandle(clientID, 'Collision#4', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for MicoHand to MicoHand#0 collision object')
    
    # Get "handle" to the MicoHand to MicoHand#0 collision object
result, collision_Handle_5 = vrep.simxGetCollisionHandle(clientID, 'Collision#5', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for collection1 collision object')
    
    
    
    result, collision_state_1 = vrep.simxReadCollision(clientID, collision_Handle_1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')
    
    result, collision_state_2 = vrep.simxReadCollision(clientID, collision_Handle_2, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')
    
    result, collision_state_3 = vrep.simxReadCollision(clientID, collision_Handle_3, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')
    
    result, collision_state_4 = vrep.simxReadCollision(clientID, collision_Handle_4, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')
    
    result, collision_state_5 = vrep.simxReadCollision(clientID, collision_Handle_5, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get collision state for robot collision object')

    
    return collision_state_5
