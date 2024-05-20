from flygym.mujoco import NeuroMechFly
from flygym.mujoco.state import KinematicPose

import matplotlib.pyplot as plt

import pickle
from pathlib import Path
import numpy as np

all_actuated_dof = ([
    f"joint_{side}{pos}{dof}"
    for side in "LR"
    for pos in "FMH" 
    for dof in [
        "Coxa",
        "Coxa_roll",
        "Coxa_yaw",
        "Femur",
        "Femur_roll",
        "Tibia",
        "Tarsus1",
    ] ]
    + [ # Antennae joints
        f"joint_{side}{dof}{angle}"
        for side in "LR"
        for dof in ["Pedicel"]
        for angle in ["", "_yaw"]
    ]  
    + [ # Abdomen joints
        f"joint_{body_name}"
        for body_name in ["A1A2", "A3", "A4", "A5", "A6"]
    ]

)
# Print all actuated DOF and their index
for index, element in enumerate(all_actuated_dof):
    print(index, element, end='\n')
print(len(all_actuated_dof))

class NeuromechflyProject(NeuroMechFly):

    def __init__(
        self,
        actuated_joints=all_actuated_dof,
        **kwargs
        ):

        super().__init__(
            actuated_joints=actuated_joints,
            **kwargs)