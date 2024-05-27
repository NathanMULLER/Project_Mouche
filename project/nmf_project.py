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

class NeuromechflyProject(NeuroMechFly):

    def _set_joints_stiffness_and_damping(self):
        super()._set_joints_stiffness_and_damping()

        for body_name in ["A1A2", "A3", "A4", "A5", "A6"]:
            body = self.model.find('body', body_name)
            
            body.add(
                "joint",
                name=f"joint_{body_name}",
                type="hinge",
                pos="0 0 0",
                axis="0 1 0",
                stiffness=5.0,
                springref=0.0,
                damping=5.0,
                dclass="nmf",
            )
            
            self.actuated_joints.append(f"joint_{body_name}")
    
    def _add_adhesion_actuators(self, gain):
        for body_name in ["A1A2", "A3", "A4", "A5", "A6"]:
            joint = self.model.find("joint", f"joint_{body_name}")
            actuator = self.model.actuator.add(
                "position",
                name=f"actuator_position_joint_{body_name}",
                joint=joint,
                forcelimited="true",
                ctrlrange="-1000000 1000000",
                forcerange = "-10 10",
                kp=self.sim_params.actuator_kp,
                dclass="nmf",
            )

            # this is needed if you do not want to override add joint sensors
            vel_actuator = self.model.actuator.add(
                "velocity",
                name=f"actuator_velocity_joint_{body_name}",
                joint=joint,
                dclass="nmf",
            )
            torque_actuator = self.model.actuator.add(
                "motor",
                name=f"actuator_torque_joint_{body_name}",
                joint=joint,
                dclass="nmf",
            )
            # self.actuated_joints.append(joint)
            self._actuators.append(actuator)

        return super()._add_adhesion_actuators(gain)
    
