# -*- coding: utf-8 -*-
"""
Created on Thu Jun 19 16:17:57 2025

"""


import zmqRemoteApi
import numpy as np


client = zmqRemoteApi.RemoteAPIClient()
sim = client.getObject('sim')

def setGripperData(open, gripperHandle, velocity=0.11, force=20):
    if not open:
        velocity = -velocity

    dat = {
        'velocity': velocity,
        'force': force
    }
    sim.writeCustomBufferData(gripperHandle, 'activity', sim.packTable(dat))

def moveToPose_viaIK(maxVelocity, maxAcceleration, maxJerk, targetQ, auxData):
    params = {
        'ik': {'tip': auxData['tip'], 'target': auxData['target'], 'base': auxData['base']},
        'targetPose': targetQ,
        'maxVel': maxVelocity,
        'maxAccel': maxAcceleration,
        'maxJerk': maxJerk,
    }
    sim.moveToPose(params)

def moveToConfig_viaFK(maxVelocity, maxAcceleration, maxJerk, goalConfig, auxData):
    params = {
        'joints': auxData['joints'],
        'targetPos': goalConfig,
        'maxVel': maxVelocity,
        'maxAccel': maxAcceleration,
        'maxJerk': maxJerk,
    }
    sim.moveToConfig(params)

"""
Get current configuration for robot joints.
"""
def getConfig(jointHandles):
    config = [0 for _ in jointHandles]
    for i, handle in enumerate(jointHandles):
        config[i] = sim.getJointPosition(handle)
    return config

def getObjectBoundingBoxSize(handle):
    s=[]
    r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_max_x)
    r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_min_x)
    s.append(m-n)
    r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_max_y)
    r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_min_y)
    s.append(m-n)
    r,m=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_max_z)
    r,n=sim.getObjectFloatParameter(handle,sim.objfloatparam_objbbox_min_z)
    s.append(m-n)
    return s

#%%
def create_heptagon_based_container(parent, container_hinges, container_side_markers, R = 0.04894, height = 0.2, thickness = 0.02):
    # REF: https://www.omnicalculator.com/math/heptagon
    # a: heptagon's side  
    # R: radius of the circumcircle
    
    a = R*( 2*np.sin(np.pi/7) )
    
    #  radius of the incircle is
    r = a / ( 2*np.tan(np.pi/7) )
    
    container_base = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [R*2,R*2,thickness])
    sim.setObjectParent(container_base, parent, True)
    sim.setObjectPosition(container_base, [0, 0, thickness/2], sim.handle_parent )
    
    
    container_sides = []
    for side_i in range(0,7):
        container_sides.append( sim.createPrimitiveShape(sim.primitiveshape_cuboid, [thickness, a,  height]) )
    
  
    #set face 0
    sim.setObjectParent(container_sides[0], container_base, True)
    sim.setObjectPosition(container_sides[0], [r+(thickness/2),0,(height/2)+(thickness/2)], container_base )
    
    sim.setObjectParent(container_hinges[0], container_sides[0], True)
    sim.setObjectPosition(container_hinges[0], [-thickness/2, a/2, 0], sim.handle_parent )
        
    sim.setObjectParent(container_side_markers[0], container_hinges[0], True)
    sim.setObjectPosition(container_side_markers[0], [0, 0, 0], sim.handle_parent )
      
    # face 1 to 5
    for i in range(1,6):
        #print(i)
        sim.setObjectParent(container_sides[i], container_side_markers[i-1], False)
        sim.setObjectPosition(container_sides[i], [thickness/2,a/2,0], sim.handle_parent )
        
        sim.setObjectParent(container_hinges[i], container_sides[i], True)
        sim.setObjectPosition(container_hinges[i], [-thickness/2, a/2, 0], sim.handle_parent )
            
        sim.setObjectParent(container_side_markers[i], container_hinges[i], True)
        sim.setObjectPosition(container_side_markers[i], [0,0, 0], sim.handle_parent )
    
    # face 6
    sim.setObjectParent(container_sides[6], container_side_markers[5], False)
    sim.setObjectPosition(container_sides[6], [thickness/2,a/2,0], sim.handle_parent )
    
    # group container parts
    hepta_container = sim.groupShapes(container_sides+[container_base])
    
    sim.setObjectInt32Parameter(hepta_container, sim.shapeintparam_static, 0)
    sim.setObjectInt32Parameter(hepta_container, sim.shapeintparam_respondable, 1)
    sim.resetDynamicObject(hepta_container)
    
    return hepta_container

#%%
def get_volume(radius, height):
    #REF: https://www.omnicalculator.com/math/heptagon
    side = radius*( 2*np.sin(np.pi/7) )
    area = (7/4)*(side*side)*(1 / np.tan(np.pi/7) )# 3.634*(side*side)#
    volume = area*height
    return volume

#%%
def get_height_from_volume(radius, volume):
    #REF: https://www.omnicalculator.com/math/heptagon
    side = radius*( 2*np.sin(np.pi/7) )
    area = (7/4)*(side*side)*(1 / np.tan(np.pi/7) )# 3.634*(side*side)#
    #volume = area*height
    height = volume/area
    return height
