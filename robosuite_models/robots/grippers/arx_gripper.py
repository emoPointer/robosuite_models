import numpy as np
from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from robosuite_models import robosuite_model_path_completion


@register_gripper
class ArxGripper(GripperModel):
    """
    ArxGripper for the ARX robotic arm (has two fingers with slide joints).
    
    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(robosuite_model_path_completion("grippers/arx_gripper.xml"), idn=idn)

    def format_action(self, action):
        """
        Format the action input to the gripper.
        
        Args:
            action (np.array): Action to be formatted
            
        Returns:
            np.array: Formatted action
        """
        assert len(action) == self.dof
        # For ARX gripper with slide joints, use position-based control
        self.current_action = np.clip(self.current_action + np.array([1.0, 1.0]) * self.speed * np.sign(action), 0.0, 1.0)  # control logic
        return self.current_action

    @property
    def init_qpos(self):
        """
        Initial joint positions for the gripper (closed position for slide joints).
        
        Returns:
            np.array: Initial joint positions in meters
        """
        return np.array([0.0, 0.0])

    @property
    def speed(self):
        """
        Speed factor for gripper actuation (adjusted for slide joints).
        
        Returns:
            float: Speed factor
        """
        return 0.01  # Reduced speed for slide joints (meters vs radians)

    @property
    def _important_geoms(self):
        """
        Important geometries for the gripper (used for collision detection and contact).
        
        Returns:
            dict: Dictionary mapping finger names to their geometry names
        """
        # Use the same pattern as UMIGripper since they have similar XML structure
        return {
            "right_fingerpad": ["right_finger_collision", "right_finger_visual"],
            "left_fingerpad": ["left_finger_visual", "left_finger_collision"],
        }

    @property
    def dof(self):
        """
        Degrees of freedom for the gripper.
        
        Returns:
            int: Number of degrees of freedom
        """
        return 2

    # @property
    # def contact_geoms(self):
    #     """
    #     Contact geometries for grasping objects.
    #     
    #     Returns:
    #         list: List of contact geometry names
    #     """
    #     return ["right_finger_collision", "left_finger_collision"]