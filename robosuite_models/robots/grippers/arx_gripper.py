# import numpy as np
# from robosuite.models.grippers import register_gripper
# from robosuite.models.grippers.gripper_model import GripperModel

# from robosuite_models import robosuite_model_path_completion


# @register_gripper
# class ArxGripper(GripperModel):
#     """
#     ArxGripper for the ARX robotic arm (1D control for both slide joints).
    
#     Control mapping:
#     -1: completely open both fingers
#      0: keep current position  
#      1: completely close both fingers
    
#     Args:
#         idn (int or str): Number or some other unique identification string for this gripper instance
#     """

#     def __init__(self, idn=0):
#         super().__init__(robosuite_model_path_completion("grippers/arx_gripper.xml"), idn=idn)
#         # Initialize current_action to fully open position (matching init_qpos)
#         # self.current_action = np.array([0.044, 0.044])  # Start fully open

#     def format_action(self, action):
#         assert len(action) == self.dof
#         self.current_action = np.clip(self.current_action + np.array([1.0]) * self.speed * np.sign(action), -1.0, 1.0)
#         return self.current_action

#     @property
#     def init_qpos(self):
#         """
#         Initial joint positions for the gripper (fully open position for slide joints).
        
#         Returns:
#             np.array: Initial joint positions for both physical joints - start fully open
#         """
#         return np.array([0.047, 0.047])  # Start at fully open position

#     @property
#     def speed(self):
#         """
#         Speed factor for gripper actuation (similar to UMI gripper).
        
#         Returns:
#             float: Speed factor for incremental control
#         """
#         return 0.15  # Moderate speed for slide joints

#     @property
#     def _important_geoms(self):
#         """
#         Important geometries for the gripper (used for collision detection and contact).
        
#         Returns:
#             dict: Dictionary mapping finger names to their geometry names
#         """
#         # Use the same pattern as UMIGripper since they have similar XML structure
#         return {
#             "right_fingerpad": ["right_finger_collision", "right_finger_visual"],
#             "left_fingerpad": ["left_finger_visual", "left_finger_collision"],
#         }

#     @property
#     def dof(self):
#         """
#         Degrees of freedom for the gripper.
        
#         Returns:
#             int: Number of degrees of freedom
#         """
#         return 1
import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class ArxGripper(GripperModel):
    """
    UMIGripper for the Arx5 arm
    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/arx_gripper.xml"), idn=idn)

    def format_action(self, action):
        assert len(action) == self.dof
        self.current_action = np.clip(self.current_action + np.array([1.0]) * self.speed * np.sign(action), -10.0, 10.0)
        return self.current_action

    @property
    def init_qpos(self):
        return np.array([-0.1, 0.1])

    @property
    def speed(self):
        return 2

    @property
    def _important_geoms(self):
        return {
            "right_fingerpad": ["right_finger_collision", "right_finger_visual"],
            "left_fingerpad": ["left_finger_visual", "left_finger_collision"],
        }
