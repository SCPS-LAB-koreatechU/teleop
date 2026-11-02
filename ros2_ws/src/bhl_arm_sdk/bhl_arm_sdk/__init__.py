"""Per-motor angle control of the Berkeley Humanoid Lite arms (recoil actuators over CAN)."""

from .config import ArmConfig, JointSpec, load_config
from .controller import ArmController

__all__ = ["ArmConfig", "ArmController", "JointSpec", "load_config"]
