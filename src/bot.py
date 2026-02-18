import math
import random
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from util.ball_prediction_analysis import find_slice_at_time

from util.drive import steer_toward_target
from util.vec import Vec3
from util.sequence import Sequence, ControlStep


class Knarr(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.current = "ball"
        self.isKickoff = False
        self.dribbleGoal = "approach"

    def initialize_agent(self):
        print("Knarr is on team: ", self.team)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        my_car = packet.game_cars[self.index]
        controls = SimpleControllerState()

        field = self.get_field_info()

        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        my_location = Vec3(my_car.physics.location)
        my_speed = Vec3(my_car.physics.velocity)
        my_rot = my_car.physics.rotation
        ball_location = Vec3(packet.game_ball.physics.location)
        # if packet.game_info.is_kickoff_pause == False:
        #     self.isKickoff = False
        # if packet.game_info.is_kickoff_pause and self.isKickoff == False:
        #     self.isKickoff = True
        #     print("begin kickoff?")
        #     self.current = "kickoff"
        # elif self.behind_ball(self.team, my_location, ball_location):
        #     # target_location = ball_location
        #     target_location = self.adjust_target(my_car, self.team, ball_location)
        #     self.current = "ball"
        #     speed = my_speed.length()
        #     if my_location.dist(ball_location) > min(1000, speed):
        #         # Estimate how long it will take to reach the ball based on current speed
        #         distance = my_location.dist(ball_location)
                
        #         # Avoid division by tiny speed which would produce a huge lookahead. Use a reasonable minimum speed.
        #         min_speed = 500.0
        #         speed_for_estimate = max(speed, min_speed)
        #         time_to_reach = distance / speed_for_estimate
        #         # Add a small buffer and clamp lookahead to a reasonable range (0.5s to 5s)
        #         lookahead = max(0.5, min(time_to_reach/2, 2.0))

        #         ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
        #         if ball_prediction is not None:
        #             # Clamp target time against the prediction's last available slice
        #             last_time = ball_prediction.slices[-1].game_seconds
        #             target_time = min(packet.game_info.seconds_elapsed + lookahead, last_time)
        #             ball_in_future = find_slice_at_time(ball_prediction, target_time)

        #             if ball_in_future is not None:
        #                 target_location = self.adjust_target(my_car, self.team, Vec3(ball_in_future.physics.location), 200)
        # else:
        #     target_location = Vec3(0, 5120 * (-1 if self.team == 0 else 1), 100)
        #     self.current = "rotate"
        
        # if self.current != "kickoff":
        #     self.renderer.draw_line_3d(my_location, target_location, self.renderer.cyan())

        #     goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
        #     self.renderer.draw_line_3d(ball_location, goal, self.renderer.red())

        #     controls.steer = steer_toward_target(my_car, target_location)
        #     controls.throttle = 1
        # steer_mag = abs(controls.steer)
        # if steer_mag < 0.5 and my_speed.length() < 2000:
        #     controls.boost = True

        last_pred = None
        target_pred = None
        timeToCatch = 0
        reachedApex = False
        for i in range(20):
            pred_slice = find_slice_at_time(self.get_ball_prediction_struct(), packet.game_info.seconds_elapsed + i * 0.1)
            if pred_slice is not None:
                if last_pred is not None:
                    self.renderer.draw_line_3d(Vec3(last_pred.physics.location), Vec3(pred_slice.physics.location), self.renderer.cyan())
                    if pred_slice.physics.location.z < last_pred.physics.location.z:
                        reachedApex = True
                    if reachedApex and pred_slice.physics.location.z <= 36 + 92.5:
                        target_pred = pred_slice
                        timeToCatch = i * 0.1

                        break
                else: 
                    self.renderer.draw_line_3d(ball_location, Vec3(pred_slice.physics.location), self.renderer.red())
            last_pred = pred_slice
        
        if target_pred is not None:
            target_location = Vec3(target_pred.physics.location)
        else:
            target_location = ball_location
        self.current = "dribble"

        flip = -1 if self.team == 0 else 1
        kickoffLocations = [
            Vec3(2048 * flip, 2560 * flip, 0), # Right
            Vec3(-2048 * flip, 2560 * flip, 0), # Left
            Vec3(256 * flip, 3840 * flip, 0), # Back Right
            Vec3(-256 * flip, 3840 * flip, 0), # Back Left
            Vec3(0, 4608 * flip, 0), # Center
            Vec3(0, 0, 5000), # fake
        ]
        if self.current == "kickoff":
            kickoffLoc = 5
            for i in range(len(kickoffLocations)):
                if my_location.dist(kickoffLocations[i]) < my_location.dist(kickoffLocations[kickoffLoc]) and my_location.dist(kickoffLocations[i]) < 1000:
                    kickoffLoc = i
            print(kickoffLoc, " pos: ", kickoffLocations[kickoffLoc], " dist: ", my_location.dist(kickoffLocations[kickoffLoc]), " my loc: ", my_location)
            self.kickoff(packet, kickoffLoc)
        elif self.current == "ball":
            if my_location.dist(target_location) < 550 and abs(controls.steer) < 0.2:
                if my_location.z < target_location.z + 100:
                    self.front_flip(packet)

                elif my_location.z < target_location.z + 500:
                    self.active_sequence = Sequence([
                        ControlStep(0.2, SimpleControllerState(jump=True)),
                        ControlStep(0.05, SimpleControllerState(jump=False)),
                        ControlStep(0.2, SimpleControllerState(jump=True, pitch=-1)),
                        ControlStep(0.8, SimpleControllerState()),
                    ])
                
                else:
                    self.active_sequence = Sequence([
                        ControlStep(0.2, SimpleControllerState(jump=True)),
                        ControlStep(0.05, SimpleControllerState(jump=False)),
                        ControlStep(0.2, SimpleControllerState(jump=True)),
                        ControlStep(0.05, SimpleControllerState()),
                    ])
        elif self.current == "rotate":
        
            if my_location.dist(target_location) > 2000 and my_speed.length() < 2000 and my_speed.length() > 500 and abs(controls.steer) < 0.3 and my_car.has_wheel_contact:
                controls.boost = False
                self.front_flip(packet)
        elif self.current == "dribble":
            # Approach
            if self.dribbleGoal == "approach":
                print("approach")
                if my_location.dist(ball_location) > 200:
                    controls.steer = steer_toward_target(my_car, ball_location)
                    controls.throttle = my_location.dist(ball_location) / 3000
                else:
                    self.dribbleGoal = "pop"
            # Pop up
            if self.dribbleGoal == "pop":
                print("pop")
                print("ball z: ", ball_location.z, " my z: ", my_location.z)
                if ball_location.z < 150 and my_location.dist(ball_location) < 300:
                    controls.steer = steer_toward_target(my_car, ball_location)
                    controls.throttle = 1
                    controls.boost = True
                else:
                    self.dribbleGoal = "catch"
            # Catch
            if self.dribbleGoal == "catch":
                print("catch")
                # Position the car under the target location (predicted catch point)
                target_under_ball = Vec3(target_location.x, target_location.y, my_location.z)
                controls.steer = steer_toward_target(my_car, target_under_ball)
                # Determine if the target is ahead or behind relative to the car
                forward = Vec3(math.cos(my_rot.yaw), math.sin(my_rot.yaw), 0)
                to_target = (target_location - my_location).flat()
                forward_dot = forward.dot(to_target)
                h_dist = to_target.length()

                # Project velocities onto the car's forward vector
                ball_vel = Vec3(packet.game_ball.physics.velocity)
                my_forward_speed = forward.dot(my_speed)
                ball_forward_speed = forward.dot(ball_vel)

                # Physics constants (uu/s and uu/s^2)
                MAX_SPEED_BOOST = 2300.0
                MAX_SPEED_NO_BOOST = 1410.0
                BOOST_ACCEL_GROUND = 991.666
                THROTTLE_ACCEL_GROUND = 350.0  # Conservative estimate for throttle-only acceleration
                
                # Calculate required speed to arrive at catch point at the right time
                if timeToCatch > 0.1:
                    required_speed = h_dist / timeToCatch
                    speed_error = required_speed - my_forward_speed
                    
                    # Determine if we need boost to achieve required speed in time
                    # Check what speed we can reach without boost
                    speed_reachable_no_boost = min(my_forward_speed + (THROTTLE_ACCEL_GROUND * timeToCatch), MAX_SPEED_NO_BOOST)
                    
                    # Only boost if:
                    # 1. Target is ahead
                    # 2. Required speed exceeds what's reachable without boost
                    # 3. Required speed is achievable with boost
                    if forward_dot > 0 and required_speed > speed_reachable_no_boost and required_speed <= MAX_SPEED_BOOST and my_car.has_wheel_contact:
                        controls.boost = True
                    else:
                        controls.boost = False
                    
                    # Cap required speed to what's actually achievable
                    max_achievable = MAX_SPEED_BOOST if controls.boost else MAX_SPEED_NO_BOOST
                    required_speed = min(required_speed, max_achievable)
                    speed_error = required_speed - my_forward_speed
                    
                    # Scale throttle based on speed error
                    throttle_cmd = max(-1.0, min(1.0, speed_error / 1500.0))
                    controls.throttle = throttle_cmd
                else:
                    # Very close to catch time, just match ball speed
                    speed_error = ball_forward_speed - my_forward_speed
                    throttle_cmd = max(-1.0, min(1.0, speed_error / 1200.0))
                    controls.throttle = throttle_cmd
                    if throttle_cmd > 0.8 and ball_forward_speed > 1400:
                        controls.boost = True
                    else:
                        controls.boost = False

                # If ball has come back down close to ground, switch to approach
                if ball_location.z < 100:
                    self.dribbleGoal = "approach"

                

        return controls
    
    def kickoff(self, packet, loc):
        if loc == 0:
            self.active_sequence = Sequence([
                ControlStep(0.15, SimpleControllerState(steer=1, throttle=1, boost=True)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=-1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=-1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
            ])
        elif loc == 1:
            self.active_sequence = Sequence([
                ControlStep(0.15, SimpleControllerState(steer=-1, throttle=1, boost=True)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
            ])
        elif loc == 2:
            self.active_sequence = Sequence([
                ControlStep(0.2, SimpleControllerState(throttle=1, steer = 0.2)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=-1, throttle=1, jump=True, boost = True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=-1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
            ])
        elif loc == 3:
            self.active_sequence = Sequence([
                ControlStep(0.2, SimpleControllerState(throttle=1, steer = -0.2)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=1, throttle=1, jump=True, boost=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False, boost=True)),
            ])
        elif loc < 5:
            return self.active_sequence.tick(packet)
        else:
            self.current = "ball"


    # Am I behind the ball?
    def behind_ball(self, team, loc, ball_loc):
        y = loc.y
        ball_y = ball_loc.y
        if team == 1:
            y = loc.y * -1
            ball_y = ball_loc.y * -1
        if y < ball_y:
            return True
        return False

    def adjust_target(self, my_car, team, ball_loc, mag = 91.25):
        goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
        direction = ball_loc - goal
        angle = math.atan2(direction.y, direction.x)
        # Offset parallel to the shot direction
        offset = Vec3(mag * math.cos(angle), mag * math.sin(angle), 0)
        target = ball_loc + offset
        self.renderer.draw_line_3d(target, ball_loc, self.renderer.red())

        self.renderer.draw_line_3d(target, Vec3(target.x + 4000 * math.cos(angle), target.y + 4000 * math.sin(angle), 0), self.renderer.purple())
        my_pos = my_car.physics.location
        my_rot = my_car.physics.rotation
        self.renderer.draw_line_3d(my_pos, Vec3(my_pos.x + 4000 * math.cos(my_rot.yaw), my_pos.y + 4000 * math.sin(my_rot.yaw), 0), self.renderer.green())
        
        car_ang_to_goal = math.atan2(goal.y - my_pos.y, goal.x - my_pos.x)
        goal_to_ball_ang = math.atan2(ball_loc.y - goal.y, ball_loc.x - goal.x)
        angle_between = abs(abs(car_ang_to_goal - goal_to_ball_ang) - math.pi)

        # print(car_ang_to_goal * 180 / math.pi, " ", goal_to_ball_ang * 180 / math.pi, " ", angle_between * 180 / math.pi)
        self.renderer.draw_string_3d(my_pos, 8,8, f"Angle: {(angle_between*180/math.pi):.1f}", self.renderer.white())
        # if abs(angle_between) < math.pi / 2:
            
            
            
        #     self.renderer.draw_line_3d(target, ball_loc, self.renderer.yellow())
        return target

    def front_flip(self, packet):

        self.active_sequence = Sequence([
            ControlStep(0.05, SimpleControllerState(jump=True)),
            ControlStep(0.05, SimpleControllerState(jump=False)),
            ControlStep(0.2, SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(0.8, SimpleControllerState()),
        ])


        return self.active_sequence.tick(packet)