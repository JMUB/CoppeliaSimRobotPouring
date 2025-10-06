#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 19 16:17:57 2025

- Run a pouring trial with parameters from the console

"""
import sys
import math
import zmqRemoteApi
import numpy as np
import pandas as pd
from tqdm import tqdm


from utils import *

client = zmqRemoteApi.RemoteAPIClient()
sim = client.getObject('sim')

#%%
# Initialize handles from simulation objects
simJoints = [sim.getObject('/joint', {'index': i}) for i in range(6)]
simTip = sim.getObject('/ikTip')
simTarget = sim.getObject('/ikTarget')
modelBase = sim.getObject('/UR5')
gripperHandle = sim.getObject('/RG2')

source_container = sim.getObject('/source_container')
source_pouring_point = sim.getObject('/source_container/source_pouring_point')
target_pouring_point = sim.getObject('/target_pouring_point')

support_target = sim.getObject('/support_target')

force_sensor = sim.getObject('/ForceSensor_target')

target_hinges = []
for  i in ['/c0b','/c1b','/c2b','/c3b','/c4b','/c5b']:
    target_hinges.append( sim.getObject( i ) )
    
target_markers = []    
for i in ['/c1m','/c2m','/c3m','/c4m','/c5m','/c6m']:
    target_markers.append( sim.getObject( i ) )


# simulate a stable grasp with connector
connector=sim.getObject('/attachPoint')

# Retrieve dimensions and poses
size_bounding_box_source_container = getObjectBoundingBoxSize(source_container)
support_target_pose = sim.getObjectPose(support_target)

height_source_container = size_bounding_box_source_container[2]
width_source_container = size_bounding_box_source_container[1]

#%%%

# velocities and accelerations in rad/s
vel = 3
accel = 1
jerk = 1.4
maxVel = [vel for _ in range(6)]
maxAccel = [accel for _ in range(6)]
maxJerk = [jerk for _ in range(6)]

grip_rot_maxVel = maxVel.copy()
#grip_rot_maxVel[5] = 8*math.pi/180

# IK movement data
ikMaxVel = [0.4]*3 + [1.8]
ikMaxAccel = [0.8]*3 + [0.9]
ikMaxJerk = [0.6]*3 + [0.8]


setGripperData(True, gripperHandle,  force=10)
sim.setInt32Param(sim.intparam_current_page, 0)

data = {
    'tip': simTip,
    'target': simTarget,
    'base': modelBase,
    'joints': simJoints
}

sim.wait(5)
source_position = sim.getObjectPosition(source_container)
pose = sim.getObjectPose(simTip)
pose[0] = source_position[0]
pose[1] = source_position[1]
pose[2] = source_position[2]
moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pose, data)
initial_config = getConfig(simJoints)

#%%
# Trial parameters:

ref_radius = width_source_container/2
ref_height = height_source_container
thickness = 0.01

# volume of the source container:
ref_volume = 0.0005142 
 
if len(sys.argv)>1:
    
    relative_diameter = float(sys.argv[1])
    
    fullness = float(sys.argv[2])
    
    relative_capacity = float(sys.argv[3])
    
    n_repetitions = int(sys.argv[4])
    
else:
    print('\nRun pouring trial with default parameters')
    relative_diameter = 1.2
    
    fullness = 0.7
    
    relative_capacity = 1
    
    n_repetitions = 1
#####################################
    
print('\nNo. replications: ', n_repetitions)


# velocities expressed in rad/s
rot_vel = 1

rot_angle = -15


# simulate marbles
particle_size = 0.015
vmarble = (4/3)*np.pi*( np.power(0.015/2, 3) )
dmarble = 0.005/vmarble

particle_density = dmarble

# dictionary of particle size to max. number of particles
part_size_to_max_n = {0.015: 175, 0.02:70, 0.025: 35,  0.03:19, 0.035:11, 0.04:7,  0.05:3}

min_particles = 1

# configure particle object properties
type = sim.particle_spheres + sim.particle_respondable1to4+ sim.particle_particlerespondable 

one_particle_force = 0.049024

# configure target container parameters
r = relative_diameter*ref_radius
    
hepta_side = r*( 2*np.tan(np.pi/7) )

# Circumcircle radius (R)
R = hepta_side/( 2*np.sin(np.pi/7) )

width_target_container = r*2

volume = ref_volume*relative_capacity
            
height_target_container = get_height_from_volume(r, volume)
    
# configure particles' properties
max_particles = part_size_to_max_n[particle_size]

particles = np.round(fullness * max_particles, 0)
if particles <1:
    particles = 1
    
#%%
results_df=pd.DataFrame()


for repetition_i in tqdm( range(0, n_repetitions) ):
    #reset pose of the support
    sim.setObjectPose(support_target, support_target_pose, sim.handle_world)
    sim.setObjectPosition(source_container, source_position, sim.handle_world )
    
    target_container = create_heptagon_based_container(force_sensor, target_hinges, target_markers,
                                           R, height_target_container, thickness)
                                           
    sim.wait(1)
    result, initial_target_force_vector, initial_target_torque_vector = sim.readForceSensor(force_sensor)        
    
    #print('repetition: ', rep )   
    p = sim.getObjectPosition(source_container)
    spawn_counter = 0

    source_particle = sim.addParticleObject(type, particle_size, particle_density, None, 0.0, 999, [1.0, 0.0, 0.0])

    while spawn_counter < particles:
        sim.addParticleObjectItem(source_particle, 
            [p[0], p[1], p[2]+0.05, p[0], p[1],p[2]+0.05])
        
        spawn_counter += 1

    
    sim.wait(1)

    # simulate stable grip:
    sim.setObjectParent(source_container,connector,True)
    
    setGripperData(False, gripperHandle)
    sim.wait(1)
    
    
    # lift arm
    pose = sim.getObjectPose(simTip)
    pose[2] += 0.15
    moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pose, data)
    #sim.wait(1)
    
    # move to pouring position
    target_postion = sim.getObjectPosition(target_container)
                
    pose = sim.getObjectPose(target_pouring_point) #NOTE: the target_pouring_dummy just provides the angles
    pose[0] = target_postion[0]-(height_source_container*0.5)-(width_target_container*0.3)
    pose[1] = target_postion[1]
    pose[2] = target_postion[2]+(height_target_container+thickness)+(height_source_container*0.6)
    

    moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pose, data)
        
    current_config = getConfig(simJoints)
    rotate_gripper_config = current_config.copy() 
    rotate_gripper_config [-1] = rot_angle*math.pi/180
    
    # rotate gripper
    grip_rot_maxVel[5] = rot_vel
    moveToConfig_viaFK(grip_rot_maxVel, maxAccel, maxJerk, rotate_gripper_config, data)
    sim.wait(5)
    
    # rotate gripper back
    rotate_gripper_config[-1] = current_config[-1]
    moveToConfig_viaFK(maxVel, maxAccel, maxJerk, rotate_gripper_config, data)
    sim.wait(1)
    
    result, target_force_vector, target_torque_vector = sim.readForceSensor(force_sensor)
    
    sim.removeParticleObject(source_particle)
    sim.removeObject(target_container)
    
    # move to initial configuration
    moveToConfig_viaFK(maxVel, maxAccel, maxJerk, initial_config, data)
    
    # detacht container to finish grasp simulation
    sim.setObjectParent(source_container,-1,True)
    setGripperData(True, gripperHandle)
    sim.wait(5)
    
    trial_df =  pd.DataFrame.from_dict({ 'n_particles': [particles],
                                        'particle_size': [particle_size],
                                        'particle_density': [particle_density],
                              })


    for axi, ax in enumerate(['x','y','z']):
        trial_df['initial_force_'+ax] = initial_target_force_vector[axi]

    for axi, ax in enumerate(['x','y','z']):
        trial_df['initial_torque_'+ax] = initial_target_torque_vector[axi]
        
    for axi, ax in enumerate(['x','y','z']):
        trial_df['final_force_'+ax] = target_force_vector[axi]

    for axi, ax in enumerate(['x','y','z']):
        trial_df['final_torque_'+ax] = target_torque_vector[axi]
        
    trial_df['initial_force'] = np.linalg.norm( 
        trial_df[['initial_force_x', 'initial_force_y', 'initial_force_z']].values, axis = 1 )
    trial_df['final_force'] = np.linalg.norm( 
        trial_df[['final_force_x', 'final_force_y', 'final_force_z']].values, axis = 1 )

    trial_df['poured_particles_force']=trial_df.final_force-trial_df.initial_force
    
    trial_df['expected_force'] = trial_df.n_particles*one_particle_force

    trial_df['estimated_particles_in_container'] =  np.round( trial_df.poured_particles_force/one_particle_force, 0)

    trial_df['spilled_particles'] = trial_df.n_particles - trial_df.estimated_particles_in_container

    trial_df['spillage_percentage'] = trial_df.spilled_particles/trial_df.n_particles

    trial_df['spillage'] = trial_df.spilled_particles>0
    
    # append manipulation data
    results_df=pd.concat([results_df, trial_df], axis=0)

                                       
sim.stopSimulation()

print('spillage in replications: ')
print(results_df['spillage'].describe())

