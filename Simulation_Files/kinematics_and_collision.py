

import numpy as np
import math
from typing import List, Tuple, Optional, Dict

class IndustrialRobotKinematics:


    def __init__(self, robot_type: str = "6DOF_INDUSTRIAL"):
        # D-H parameters for a 6-DOF paint / handling robot (approx 2.0m reach)
        # alpha in rad, a in m, d in m
        self.dh_params = [
            {'alpha': 0.0,         'a': 0.0,    'd': 0.675, 'min_limit': -math.pi,        'max_limit': math.pi},         # J1 (Waist)
            {'alpha': -math.pi/2,  'a': 0.260,  'd': 0.0,   'min_limit': -math.radians(100), 'max_limit': math.radians(145)},# J2 (Shoulder)
            {'alpha': 0.0,         'a': 0.680,  'd': 0.0,   'min_limit': -math.radians(210), 'max_limit': math.radians(65)}, # J3 (Elbow)
            {'alpha': -math.pi/2,  'a': -0.035, 'd': 0.670, 'min_limit': -math.radians(350), 'max_limit': math.radians(350)},# J4 (Wrist 1)
            {'alpha': math.pi/2,   'a': 0.0,    'd': 0.0,   'min_limit': -math.radians(130), 'max_limit': math.radians(130)},# J5 (Wrist 2)
            {'alpha': -math.pi/2,  'a': 0.0,    'd': 0.158, 'min_limit': -math.radians(350), 'max_limit': math.radians(350)} # J6 (Flange)
        ]

        # Tool offset (Paint spray gun extension along Z-axis)
        self.tool_offset_z = 0.22  # 220 mm gun length

        # Base Position in World Coordinates (Cell layout)
        self.base_position = np.array([0.0, 2.3, 0.0])  # Mounted alongside car on left side

    @staticmethod
    def dh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
        
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ])

    def forward_kinematics(self, joint_angles: List[float]) -> Tuple[np.ndarray, List[np.ndarray]]:
        
        T = np.eye(4)
        T[0:3, 3] = self.base_position
        link_positions = [T[0:3, 3].copy()]

        for i, q in enumerate(joint_angles):
            dh = self.dh_params[i]
            T_i = self.dh_transform(dh['alpha'], dh['a'], dh['d'], q)
            T = np.dot(T, T_i)
            link_positions.append(T[0:3, 3].copy())

        # Add end-effector tool offset
        T_tool = np.eye(4)
        T_tool[2, 3] = self.tool_offset_z
        T_tcp = np.dot(T, T_tool)
        link_positions.append(T_tcp[0:3, 3].copy())

        return T_tcp, link_positions

    def check_joint_limits(self, joint_angles: List[float]) -> Tuple[bool, List[str]]:
        
        violations = []
        is_valid = True
        for i, q in enumerate(joint_angles):
            min_lim = self.dh_params[i]['min_limit']
            max_lim = self.dh_params[i]['max_limit']
            if q < min_lim or q > max_lim:
                is_valid = False
                violations.append(f"Joint J{i+1} out of bounds: {math.degrees(q):.1f} deg (Limits: [{math.degrees(min_lim):.1f}, {math.degrees(max_lim):.1f}])")
        return is_valid, violations

class CellCollisionChecker:
    

    def __init__(self):
        # Car Body Axis-Aligned Bounding Boxes (AABBs) in meters: [min_x, min_y, min_z, max_x, max_y, max_z]
        self.car_obstacles = [
            # Main Cabin
            {'name': 'Car_Cabin', 'box': np.array([-1.2, -0.95, 0.4, 0.8, 0.95, 1.6])},
            # Hood
            {'name': 'Car_Hood', 'box': np.array([0.8, -0.90, 0.3, 2.4, 0.90, 1.05])},
            # Trunk
            {'name': 'Car_Trunk', 'box': np.array([-2.4, -0.90, 0.3, -1.2, 0.90, 1.1])},
            # Conveyor / Support Skid
            {'name': 'Conveyor_Skid', 'box': np.array([-3.0, -0.8, 0.0, 3.0, 0.8, 0.3])}
        ]

        # Minimum safety clearance between robot arm links and car body
        self.safety_margin = 0.05  # 50 mm

    @staticmethod
    def point_inside_box(point: np.ndarray, box: np.ndarray, margin: float = 0.0) -> bool:
        return (box[0] - margin <= point[0] <= box[3] + margin and
                box[1] - margin <= point[1] <= box[4] + margin and
                box[2] - margin <= point[2] <= box[5] + margin)

    @staticmethod
    def segment_box_intersection(p1: np.ndarray, p2: np.ndarray, box: np.ndarray, margin: float = 0.0) -> bool:
        num_samples = 10
        for t in np.linspace(0, 1, num_samples):
            pt = p1 + t * (p2 - p1)
            if CellCollisionChecker.point_inside_box(pt, box, margin):
                return True
        return False

    def check_robot_collision(self, link_positions: List[np.ndarray], tcp_is_painting: bool = True) -> Tuple[bool, str]:
        
        # Iterate over each robot link segment
        for i in range(len(link_positions) - 1):
            p1 = link_positions[i]
            p2 = link_positions[i + 1]
            is_tool_segment = (i == len(link_positions) - 2)

            for obs in self.car_obstacles:
                # If tool is actively painting, allow standoff clearance check without false-positive alarm
                margin = self.safety_margin if not is_tool_segment else 0.01
                if self.segment_box_intersection(p1, p2, obs['box'], margin=margin):
                    return True, f"Collision detected between Link {i}->{i+1} and {obs['name']}"

        return False, "Collision-Free"

if __name__ == '__main__':
    fk_engine = IndustrialRobotKinematics()
    col_engine = CellCollisionChecker()

    test_q = [0.0, math.radians(20), math.radians(45), 0.0, math.radians(30), 0.0]
    tcp_pose, link_pts = fk_engine.forward_kinematics(test_q)
    limits_ok, viol = fk_engine.check_joint_limits(test_q)
    collision, msg = col_engine.check_robot_collision(link_pts)

    print("==================================================")
    print(" KINEMATICS & COLLISION CHECK REPORT")
    print("==================================================")
    print(f" Joint Configuration : {[round(math.degrees(q), 1) for q in test_q]} deg")
    print(f" Joint Limits Status : {'PASS' if limits_ok else 'FAIL: ' + str(viol)}")
    print(f" End-Effector TCP Pos: X={tcp_pose[0,3]:.3f}m, Y={tcp_pose[1,3]:.3f}m, Z={tcp_pose[2,3]:.3f}m")
    print(f" Collision Status    : {msg} ({'SAFE' if not collision else 'ALERT'})")
    print("==================================================")
