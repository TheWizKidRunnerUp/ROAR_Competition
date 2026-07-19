"""
Competition instructions:
Please do not change anything else but fill out the to-do sections.
"""

from typing import List, Tuple, Dict, Optional
import roar_py_interface
import numpy as np

def normalize_rad(rad : float):
    return (rad + np.pi) % (2 * np.pi) - np.pi

def filter_waypoints(location : np.ndarray, current_idx: int, waypoints : List[roar_py_interface.RoarPyWaypoint]) -> int:
    def dist_to_waypoint(waypoint : roar_py_interface.RoarPyWaypoint):
        return np.linalg.norm(
            location[:2] - waypoint.location[:2]
        )
    for i in range(current_idx, len(waypoints) + current_idx):
        if dist_to_waypoint(waypoints[i%len(waypoints)]) < 3:
            return i % len(waypoints)
    return current_idx

class RoarCompetitionSolution:
    def __init__(
        self,
        maneuverable_waypoints: List[roar_py_interface.RoarPyWaypoint],
        vehicle : roar_py_interface.RoarPyActor,
        camera_sensor : roar_py_interface.RoarPyCameraSensor = None,
        location_sensor : roar_py_interface.RoarPyLocationInWorldSensor = None,
        velocity_sensor : roar_py_interface.RoarPyVelocimeterSensor = None,
        rpy_sensor : roar_py_interface.RoarPyRollPitchYawSensor = None,
        occupancy_map_sensor : roar_py_interface.RoarPyOccupancyMapSensor = None,
        collision_sensor : roar_py_interface.RoarPyCollisionSensor = None,
    ) -> None:
        self.maneuverable_waypoints = maneuverable_waypoints
        self.vehicle = vehicle
        self.camera_sensor = camera_sensor
        self.location_sensor = location_sensor
        self.velocity_sensor = velocity_sensor
        self.rpy_sensor = rpy_sensor
        self.occupancy_map_sensor = occupancy_map_sensor
        self.collision_sensor = collision_sensor
    
    async def initialize(self) -> None:
        # TODO: You can do some initial computation here if you want to.
        # For example, you can compute the path to the first waypoint.

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()



        self.current_waypoint_idx = 10
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )
        self.straight_ticks = 0
        self.straight_ticks = 0
        self.s_turn_ticks = 0
        self.slow_s_turn_active = False
        self.lookahead_distance = 0


    async def step(
        self
    ) -> None:
        """
        This function is called every world step.
        Note: You should not call receive_observation() on any sensor here, instead use get_last_observation() to get the last received observation.
        You can do whatever you want here, including apply_action() to the vehicle.
        """
        # TODO: Implement your solution here.

        # Receive location, rotation and velocity data 
        vehicle_location = self.location_sensor.get_last_gym_observation()
        vehicle_rotation = self.rpy_sensor.get_last_gym_observation()
        vehicle_velocity = self.velocity_sensor.get_last_gym_observation()
        vehicle_velocity_norm = np.linalg.norm(vehicle_velocity)

        
        
        # Find the waypoint closest to the vehicle
        self.current_waypoint_idx = filter_waypoints(
            vehicle_location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )
         # We use the 3rd waypoint ahead of the current waypoint as the target waypoint
        lookahead_distance = int(3+vehicle_velocity_norm*0.35)
        if self.s_turn_ticks > 0 and self.slow_s_turn_active:
            #lookahead_distance = min(lookahead_distance, 12)
            lookahead_distance = lookahead_distance
        self.lookahead_distance = lookahead_distance
        waypoint_to_follow = self.maneuverable_waypoints[(self.current_waypoint_idx + lookahead_distance) % len(self.maneuverable_waypoints)]

        # Calculate delta vector towards the target waypoint
        vector_to_waypoint = (waypoint_to_follow.location - vehicle_location)[:2]
        heading_to_waypoint = np.arctan2(vector_to_waypoint[1],vector_to_waypoint[0])

        # Calculate delta angle towards the target waypoint
        delta_heading = normalize_rad(heading_to_waypoint - vehicle_rotation[2])
        self.delta_heading = delta_heading

        # Proportional controller to steer the vehicle towards the target waypoint

        

       

        waypoints = self.maneuverable_waypoints
        n = len(waypoints)
        i = self.current_waypoint_idx
        segment = 30 

        s_start = 60  # begin checking roughly 80 m ahead

        p0 = waypoints[(i + s_start) % n].location[:2]
        p1 = waypoints[(i + s_start + segment) % n].location[:2]
        p2 = waypoints[(i + s_start + 2 * segment) % n].location[:2]
        p3 = waypoints[(i + s_start + 3 * segment) % n].location[:2]

        heading1 = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
        heading2 = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        heading3 = np.arctan2(p3[1] - p2[1], p3[0] - p2[0])

        turn1 = normalize_rad(heading2 - heading1)
        turn2 = normalize_rad(heading3 - heading2)

        is_s_turn = (
            turn1 * turn2 < 0
            and abs(turn1) > 0.12
            and abs(turn2) > 0.12
            and (abs(turn1) + abs(turn2)) > 0.35
        )




        # Close-range corner detection
        waypoint_to_follow_close = self.maneuverable_waypoints[(self.current_waypoint_idx + 6) % len(self.maneuverable_waypoints)]

        vector_to_waypoint_close = (waypoint_to_follow_close.location - vehicle_location)[:2]
        heading_to_waypoint_close = np.arctan2(vector_to_waypoint_close[1],vector_to_waypoint_close[0])
        delta_heading_close = normalize_rad(heading_to_waypoint_close - vehicle_rotation[2])
        self.delta_heading_close = delta_heading_close

        high_speed_threshold = 190
        middle_speed_threshold = 190
        low_speed_threshold = 60
        if abs(delta_heading_close) > 0.08:
            target_speed = low_speed_threshold
        elif abs(delta_heading_close) > 0.025:
            target_speed = middle_speed_threshold
        else:
            target_speed = high_speed_threshold

        # Very long-distance corner detection
        long_lookahead_far = 100

        waypoint_to_follow_far = self.maneuverable_waypoints[
            (self.current_waypoint_idx + long_lookahead_far)
            % len(self.maneuverable_waypoints)
        ]

        vector_to_waypoint_far = (
            waypoint_to_follow_far.location - vehicle_location
        )[:2]

        heading_to_waypoint_far = np.arctan2(
            vector_to_waypoint_far[1],
            vector_to_waypoint_far[0]
        )

        delta_heading_far = normalize_rad(
            heading_to_waypoint_far - vehicle_rotation[2]
        )
        self.delta_heading_far = delta_heading_far

        if abs(delta_heading_far) > 0.4:
            target_speed = min(target_speed, low_speed_threshold)
        elif abs(delta_heading_far) > 0.06:
            target_speed = min(target_speed, middle_speed_threshold)


        if self.s_turn_ticks > 0:
            
            self.speed_mode = "special-med"
        elif self.slow_s_turn_active == True:
            self.speed_mode = "slow-med"
        elif target_speed <= low_speed_threshold:
            self.speed_mode = "low"
        elif target_speed <= middle_speed_threshold:
            self.speed_mode = "middle"
        else:
            self.speed_mode = "high"

        
        if is_s_turn and self.s_turn_ticks == 0:
            self.slow_s_turn_active = vehicle_velocity_norm >= 71.0

        if is_s_turn:
            self.s_turn_ticks = 80
        elif self.s_turn_ticks > 0:
            self.s_turn_ticks -= 1
            if self.s_turn_ticks == 0:
                self.slow_s_turn_active = False

        if self.s_turn_ticks > 0 and self.slow_s_turn_active:
            target_speed = min(target_speed, 45)
        elif self.s_turn_ticks > 0:
            target_speed = min(target_speed, 52)

       

        

        near_turn = abs(delta_heading)
        far_turn = abs(delta_heading_far)
        if self.s_turn_ticks > 0:
            steering_gain = 32.0
        elif far_turn > 0.4:
            steering_gain = 25.0
        elif far_turn > 0.06:
            steering_gain = 22.0
        else:
            steering_gain = 19.0    # normal driving
        self.steering_gain = steering_gain



        steer_control = (
            -steering_gain / np.sqrt(vehicle_velocity_norm) * delta_heading / np.pi
        ) if vehicle_velocity_norm > 1e-2 else -np.sign(delta_heading)
        steer_control = np.clip(steer_control, -1.0, 1.0)


        self.target_speed = target_speed

        speed_control = 0.05 * (target_speed - vehicle_velocity_norm)
        throttle_control = np.clip(speed_control, 0.0, 1.0)
        brake_control = np.clip(-speed_control, 0.0, 1.0)



        control = {
            "throttle": throttle_control,
            "steer": steer_control,
            "brake": brake_control,
            "hand_brake": 0.0,
            "reverse": 0,
            "target_gear": 0
        }
        await self.vehicle.apply_action(control)
        return control
