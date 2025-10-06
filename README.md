# CoppeliaSimRobotPouring

__NOTE__: This repository is not actively maintained.  

Robot pouring simulation with a UR5 robot arm and a parallel jaw gripper. The simulation was implemented using the robotics simulator CoppeliaSim Version 4.8.0 (rev. 0) with
the Open Dynamics Engine (ODE). 

In the simulation, the robot pours simulated marbles, represented by particles, from a source container into a target container. 

![Robot pouring simulation](robot_pouring_simulation_trial_examples.gif)

The __CoppeliaSim__ simulator is free to use, in the form of an educational version, by students, teachers and professors alike belonging to a University. We provide the python script
running the pouring simulation in CoppeliaSim and the corresponding file describing the robot environment.

# INSTRUCTIONS
- Create a Python environment using the requirements.txt file 
- Open the robot_pouring.ttt CoppeliaSim scene file
- Activate the Python environment and execute the script specifying the following parameters:
  - "relative_diameter":  represents the relation between the container rim diameters, defined as *RD = target diameter/source diameter*. RD < 1 corresponds to a target container of smaller diameter, and RD > 1 corresponds to a target container of larger diameter.     
   - "fullness": FU expresses the fullness level of the source container as a fraction. For example, FU = 0.5 corresponds to a half-full source container.     
  - "relative_capacity": represents the relation between the containers' capacities, defined as  *RC = target capacity/source capacity*. RC < 1 corresponds to a target container with lower capacity, and RC > 1 to a target container with larger capacity.
   - "n_repetitions": number of trial repetitions.

Note that the target container's height and diameter are determined based on RC and RD.
