from dataclasses import dataclass
from typing import Dict, List

STEPS_PER_CM = 110.65

@dataclass
class Points:
    x: List[float]
    y: List[float]
    z: List[float]
    vx: Dict[int, float]
    vy: Dict[int, float]
    vz: Dict[int, float]
    length: int

@dataclass
class MotorParams:
    speed: float
    acceleration: float
    init_steps_to_hold_speed: int
    init_steps_to_brake: int
