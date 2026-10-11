

import numpy as np
import math
from typing import List, Dict, Tuple

class SurfaceTrajectoryPlanner:
    

    def __init__(self, standoff_distance: float = 0.25, line_spacing: float = 0.15, step_resolution: float = 0.05):
        
        self.standoff = standoff_distance
        self.line_spacing = line_spacing
        self.step_resolution = step_resolution

        # Reference Car Dimensions (in meters)
        self.car_length = 4.80  # X-axis (-2.4 to +2.4)
        self.car_width = 1.90   # Y-axis (-0.95 to +0.95)
        self.car_height = 1.60  # Z-axis (0.2 to 1.8)

    def generate_raster_grid(self, u_range: Tuple[float, float], v_range: Tuple[float, float], 
                             surface_evaluator, fixed_axis: str = 'z') -> List[Dict]:
        
        u_min, u_max = u_range
        v_min, v_max = v_range

        v_steps = int(math.ceil((v_max - v_min) / self.line_spacing)) + 1
        v_coords = np.linspace(v_min, v_max, v_steps)

        waypoints = []
        reverse_direction = False

        for v in v_coords:
            u_steps = int(math.ceil((u_max - u_min) / self.step_resolution)) + 1
            u_line = np.linspace(u_min, u_max, u_steps)
            
            if reverse_direction:
                u_line = u_line[::-1]

            for u in u_line:
                pos, normal, euler_angles = surface_evaluator(u, v, self.standoff)
                waypoints.append({
                    'position': pos,
                    'normal': normal,
                    'orientation': euler_angles,  # [Roll, Pitch, Yaw] in radians
                    'spray_active': True
                })

            reverse_direction = not reverse_direction

        return waypoints

    def plan_hood_trajectory(self) -> List[Dict]:
        
        def eval_hood(x, y, standoff):
            # Gentle curvature profile
            z_surf = 0.95 - 0.08 * ((x - 0.8) / 1.3)**2 - 0.05 * (y / 0.8)**2
            pos = [x, y, z_surf + standoff]
            # Normal pointing upwards (+Z) with slight forward tilt
            normal = [0.08, 0.0, 1.0]
            norm_mag = math.sqrt(sum(n**2 for n in normal))
            normal = [n / norm_mag for n in normal]
            
            # Gun pointing downwards (-Z)
            # Euler angles: Roll = 0, Pitch = 180 deg (facing down), Yaw = 0
            euler = [math.pi, 0.0, 0.0]
            return pos, normal, euler

        return self.generate_raster_grid(u_range=(0.9, 2.1), v_range=(-0.75, 0.75), surface_evaluator=eval_hood)

    def plan_roof_trajectory(self) -> List[Dict]:
        
        def eval_roof(x, y, standoff):
            z_surf = 1.60 - 0.04 * (y / 0.75)**2
            pos = [x, y, z_surf + standoff]
            normal = [0.0, 0.0, 1.0]
            euler = [math.pi, 0.0, 0.0]
            return pos, normal, euler

        return self.generate_raster_grid(u_range=(-1.0, 0.5), v_range=(-0.70, 0.70), surface_evaluator=eval_roof)

    def plan_left_side_trajectory(self) -> List[Dict]:
        
        def eval_left_side(x, z, standoff):
            y_surf = 0.95
            pos = [x, y_surf + standoff, z]
            normal = [0.0, 1.0, 0.0]
            # Gun pointing towards -Y (into the vehicle side)
            euler = [math.pi / 2, 0.0, 0.0]
            return pos, normal, euler

        # Here u = X, v = Z
        return self.generate_raster_grid(u_range=(-1.7, 1.7), v_range=(0.4, 1.3), surface_evaluator=eval_left_side)

    def plan_right_side_trajectory(self) -> List[Dict]:
        
        def eval_right_side(x, z, standoff):
            y_surf = -0.95
            pos = [x, y_surf - standoff, z]
            normal = [0.0, -1.0, 0.0]
            # Gun pointing towards +Y (into the right side)
            euler = [-math.pi / 2, 0.0, 0.0]
            return pos, normal, euler

        return self.generate_raster_grid(u_range=(-1.7, 1.7), v_range=(0.4, 1.3), surface_evaluator=eval_right_side)

    def plan_trunk_trajectory(self) -> List[Dict]:
        
        def eval_trunk(x, y, standoff):
            z_surf = 1.05 - 0.05 * ((x + 1.3) / 0.9)**2
            pos = [x, y, z_surf + standoff]
            normal = [-0.1, 0.0, 1.0]
            norm_mag = math.sqrt(sum(n**2 for n in normal))
            normal = [n / norm_mag for n in normal]
            euler = [math.pi, 0.0, 0.0]
            return pos, normal, euler

        return self.generate_raster_grid(u_range=(-2.1, -1.4), v_range=(-0.70, 0.70), surface_evaluator=eval_trunk)

    def generate_full_cell_program(self) -> Dict[str, List[Dict]]:
        
        # Home & Transition poses
        home_pose = {
            'position': [0.0, 1.8, 2.2],
            'orientation': [math.pi, 0.0, 0.0],
            'spray_active': False
        }

        program = {
            'HOOD': self.plan_hood_trajectory(),
            'ROOF': self.plan_roof_trajectory(),
            'LEFT_SIDE': self.plan_left_side_trajectory(),
            'RIGHT_SIDE': self.plan_right_side_trajectory(),
            'TRUNK': self.plan_trunk_trajectory()
        }

        return program

if __name__ == '__main__':
    planner = SurfaceTrajectoryPlanner(standoff_distance=0.25, line_spacing=0.15)
    full_prog = planner.generate_full_cell_program()
    print("==================================================")
    print(" ROBOTIC CAR PAINTING TRAJECTORY SUMMARY")
    print("==================================================")
    total_waypoints = sum(len(wps) for wps in full_prog.values())
    for surface, wps in full_prog.items():
        print(f" Surface: {surface:<12} | Waypoints: {len(wps):>4} | Start: {wps[0]['position']} -> End: {wps[-1]['position']}")
    print(f" Total Trajectory Waypoints: {total_waypoints}")
    print("==================================================")
