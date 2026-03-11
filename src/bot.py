import math
import random
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from util.ball_prediction_analysis import find_slice_at_time

from util.drive import steer_toward_target
from util.vec import Vec3
from util.sequence import Sequence, ControlStep

from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator, GameInfoState



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
        target_location = ball_location

        behindDist = self.behind_ball(self.team, my_location, ball_location, True)
        if packet.game_info.is_kickoff_pause == False:
            self.isKickoff = False
        if packet.game_info.is_kickoff_pause and self.isKickoff == False:
            self.isKickoff = True
            print("begin kickoff?")
            self.current = "kickoff"
        elif (self.current == "kickoff" or self.current == "rotate") and self.behind_ball(self.team, my_location, ball_location, True) > 200:
            # target_location = ball_location
            target_location = self.adjust_target(my_car, self.team, ball_location)
            self.current = "ball"
            speed = my_speed.length()
            # if my_location.dist(ball_location) > min(1000, speed):
            #     # Estimate how long it will take to reach the ball based on current speed
            #     distance = my_location.dist(ball_location)
                
            #     # Avoid division by tiny speed which would produce a huge lookahead. Use a reasonable minimum speed.
            #     min_speed = 100.0
            #     speed_for_estimate = max(speed, min_speed)
            #     time_to_reach = distance / speed_for_estimate
            #     # Add a small buffer and clamp lookahead to a reasonable range (0.5s to 5s)
            #     lookahead = max(0.5, min(time_to_reach/2, 2.0))

            #     ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
            #     if ball_prediction is not None:
            #         # Clamp target time against the prediction's last available slice
            #         last_time = ball_prediction.slices[-1].game_seconds
            #         target_time = min(packet.game_info.seconds_elapsed + lookahead, last_time)
            #         ball_in_future = find_slice_at_time(ball_prediction, target_time)

            #         if ball_in_future is not None:
            #             target_location = self.adjust_target(my_car, self.team, Vec3(ball_in_future.physics.location), 200)
        
        if self.current != "kickoff":
            self.renderer.draw_line_3d(my_location, target_location, self.renderer.cyan())

            goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
            self.renderer.draw_line_3d(ball_location, goal, self.renderer.red())

            controls.steer = steer_toward_target(my_car, target_location)
            controls.throttle = 1
        steer_mag = abs(controls.steer)
        if steer_mag < 0.5 and my_speed.length() < 2000:
            controls.boost = True

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
                    if pred_slice.physics.location.z <= 36 + 92.5:
                        ball_pred_speed = Vec3(pred_slice.physics.velocity).length()
                        speed_diff = abs(ball_pred_speed - my_speed.length())
                        if reachedApex or speed_diff < 100:
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
            print(my_location.dist(target_location))
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
            else:
                if my_location.dist(target_location) < 1000:
                    self.current = "dribble"
        elif self.current == "rotate":
            target_location = Vec3(my_location.x/abs(my_location.x) * 893, 5120 * (-1 if self.team == 0 else 1), 100)
            controls.steer = steer_toward_target(my_car, target_location)
            if my_location.dist(target_location) > 3500 and my_speed.length() < 2000 and my_speed.length() > 500 and abs(controls.steer) < 0.3 and my_car.has_wheel_contact:
                controls.boost = False
                self.front_flip(packet)
            elif abs(controls.steer) < 0.3:
                controls.boost = True
        elif self.current == "dribble":
            if behindDist < -200: 
                self.current = "rotate"

            if self.dribbleGoal == "rotate":
                goal_direction = -1 if self.team == 0 else 1
                rotateTarget = Vec3(my_car.physics.location) + Vec3(0, 200 * goal_direction, 0)
                self.renderer.draw_line_3d(my_location, rotateTarget, self.renderer.orange())
                # Steer toward a point in front of the car in the direction of the goal
                
                controls.throttle = 1
                controls.steer = steer_toward_target(my_car, rotateTarget)
                if behindDist > 300:
                    self.dribbleGoal = "approach"
            # Approach
            if self.dribbleGoal == "approach":
                if my_location.dist(ball_location) > 200:
                    controls.steer = steer_toward_target(my_car, ball_location)
                    controls.throttle = my_location.dist(ball_location) / 3000
                elif my_location.dist(ball_location) > 400:
                    controls.steer = steer_toward_target(my_car, ball_location)
                    controls.throttle = 1
                    controls.boost = True
                else:
                    self.dribbleGoal = "pop"

            # Pop up
            if self.dribbleGoal == "pop":
                GOAL_POST_DISTANCE = 892.755
                
                # Extrapolate ball position to goal line based on current velocity
                ball_vel = Vec3(packet.game_ball.physics.velocity)
                if abs(ball_vel.y) > 100:  # Only if moving toward goal
                    # Time to reach goal y-coordinate
                    goal_y = -5120 * (-1 if self.team == 1 else 1)
                    time_to_goal = abs(goal_y - target_location.y) / abs(ball_vel.y)
                    # Extrapolate x position at goal
                    extrapolated_x = target_location.x + ball_vel.x * time_to_goal
                    is_on_goal_line = abs(extrapolated_x) <= GOAL_POST_DISTANCE
                if is_on_goal_line:
                    self.current = "rotate"

                # print("ball z: ", ball_location.z, " my z: ", my_location.z)
                if ball_location.z < 150 and my_location.dist(ball_location) < 300:
                    controls.steer = steer_toward_target(my_car, ball_location)
                    controls.throttle = 1
                    controls.boost = True
                else:
                    self.dribbleGoal = "catch"
            # Catch
            if self.dribbleGoal == "catch":
                # print("catch", ball_location.z)
                if ball_location.z < 100:
                    self.dribbleGoal = "approach"
                    # ball_state = BallState(Physics(location=Vector3(0, 0, 500), velocity=Vector3(0,0,10)))
                    # ball_state = BallState(Physics(velocity=Vector3(0,0,200)))

                    # self.set_game_state(GameState(ball=ball_state))

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

                # Find a point perpendicular to the line from ball to goal
                goal = Vec3(0, 5120 * (-1 if self.team == 1 else 1), 0)
                ball_to_goal = (goal - target_location).flat().normalized()
                # Perpendicular vector (rotate 90 degrees counterclockwise)
                perpendicular = Vec3(-ball_to_goal.y, ball_to_goal.x, 0)
                
                # Ensure perpendicular points away from car (opposite side of ball-to-goal line)
                ball_to_car = (my_location - ball_location).flat()
                if perpendicular.dot(ball_to_car) > 0:
                    perpendicular = perpendicular * -1
                
                # Initialize perpendicular direction if not set
                if not hasattr(self, 'perp_direction'):
                    self.perp_direction = perpendicular
                    self.last_perp_update_time = 0
                
                # Only update perpendicular direction occasionally to prevent jitter
                current_time = packet.game_info.seconds_elapsed
                if current_time - self.last_perp_update_time > 0.2:  # Update every 200ms
                    # Check if new direction is significantly different
                    alignment = perpendicular.dot(self.perp_direction)
                    if alignment < 0.8:  # ~36 degree threshold
                        self.perp_direction = perpendicular
                        self.last_perp_update_time = current_time
                
                # Only adjust dribble if ball trajectory is not already on line to score
                # Goal posts are at ±892.755 from center
                GOAL_POST_DISTANCE = 892.755
                
                # Extrapolate ball position to goal line based on current velocity
                ball_vel = Vec3(packet.game_ball.physics.velocity)
                if abs(ball_vel.y) > 100:  # Only if moving toward goal
                    # Time to reach goal y-coordinate
                    goal_y = 5120 * (-1 if self.team == 1 else 1)
                    time_to_goal = abs(goal_y - target_location.y) / abs(ball_vel.y)
                    # Extrapolate x position at goal
                    extrapolated_x = target_location.x + ball_vel.x * time_to_goal
                    is_on_goal_line = abs(extrapolated_x) <= GOAL_POST_DISTANCE
                else:
                    is_on_goal_line = False
                
                # Calculate desired offset (updates every frame with ball movement)
                if is_on_goal_line:
                    # Already aligned to score, don't adjust position
                    desired_offset = Vec3(0, 0, 0)
                    print("on target")
                else:
                    print("off target")
                    # Scale offset based on angle change needed to align ball with goal
                    # Larger angle mismatch = larger offset needed to reposition
                    angle_to_goal = math.atan2(goal.y - target_location.y, goal.x - target_location.x)
                    perp_angle = math.atan2(perpendicular.y, perpendicular.x)
                    angle_diff = abs(angle_to_goal - perp_angle)
                    # Normalize angle difference to 0-1 range (π radians = max difference)
                    angle_factor = min(abs(angle_diff) / (math.pi / 2), 2.0)
                    
                    # Add hysteresis to perpendicular direction to prevent flipping
                    if not hasattr(self, 'last_perp_direction'):
                        self.last_perp_direction = self.perp_direction
                    
                    # Only change direction if the dot product indicates significant misalignment
                    perp_alignment = self.perp_direction.dot(self.last_perp_direction)
                    if perp_alignment < 0.7:  # ~45 degree threshold
                        self.last_perp_direction = self.perp_direction
                    
                    desired_offset = self.last_perp_direction * min(abs(timeToCatch-2) * 15.0 * angle_factor, 100.0)  # Cap maximum offset at 100 units
                    
                    # Ensure the offset doesn't put us too far behind the ball
                    ball_to_offset = desired_offset.flat()
                    ball_to_car = (my_location - target_location).flat()
                    
                    # If the offset would put us far behind the ball, reduce it
                    if ball_to_offset.dot(ball_to_car) < 0:  # Offset is behind ball relative to car
                        # Scale down the offset if it would put us too far back
                        max_behind_distance = 150.0  # Don't go more than 150 units behind the ball
                        current_behind_distance = abs(ball_to_offset.dot(ball_to_car.normalized()))
                        if current_behind_distance > max_behind_distance:
                            scale_factor = max_behind_distance / current_behind_distance
                            desired_offset = desired_offset * scale_factor
                
                # Smooth the offset changes to reduce jittery movement
                if not hasattr(self, 'smoothed_offset'):
                    self.smoothed_offset = desired_offset
                    
                offset_change = desired_offset - self.smoothed_offset
                max_offset_change_per_frame = 30.0  # Limit offset change per frame
                if offset_change.length() > max_offset_change_per_frame:
                    offset_change = offset_change.normalized() * max_offset_change_per_frame
                self.smoothed_offset = self.smoothed_offset + offset_change
                
                # Final target follows ball every frame but with smoothed offset
                adjusted_target = target_location + self.smoothed_offset
                
                # print(timeToCatch)
                adjusted_target.z = my_location.z
                
                self.renderer.draw_line_3d(target_location, goal, self.renderer.purple())
                self.renderer.draw_line_3d(target_location, adjusted_target, self.renderer.yellow())

                # if(timeToCatch < 0.4):
                # Calculate steering with reduced aggressiveness when close to ball
                dist_to_adjusted = my_location.dist(adjusted_target)
                if dist_to_adjusted < 200:  # Close to target, be gentle
                    steer_input = steer_toward_target(my_car, adjusted_target) * 0.3
                else:
                    steer_input = steer_toward_target(my_car, adjusted_target)
                controls.steer = steer_input
                # print("adjusted catch CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC")

                # Physics constants (uu/s and uu/s^2)
                MAX_SPEED_BOOST = 2300.0
                MAX_SPEED_NO_BOOST = 1410.0
                BOOST_ACCEL_GROUND = 991.666
                THROTTLE_ACCEL_GROUND = 350.0  # Conservative estimate for throttle-only acceleration
                
                # Calculate required speed to arrive at catch point at the right time
                speed_error = ball_forward_speed - my_forward_speed
                # print(speed_error)

                if abs(speed_error) < 100 and ball_location.dist(goal) < 1000:
                    self.flick(my_car, ball_location, goal)

                if timeToCatch > 0.2:
                    print("far catch")
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
                # elif abs(speed_error) > 1:
                #     print("close catch  c..()")
                #     # Very close to catch time, just match ball speed
                #     speed_error = ball_forward_speed - my_forward_speed
                #     throttle_cmd = max(-1.0, min(1.0, speed_error / 1200.0))
                #     controls.throttle = throttle_cmd
                #     if throttle_cmd > 0.9 and ball_forward_speed > 1400:
                #         controls.boost = True
                #     else:
                #         controls.boost = False
                else:

                    # Very close to catch time, just match ball speed
                    speed_error = ball_forward_speed - my_forward_speed
                    throttle_cmd = max(-1.0, min(1.0, speed_error / 1200.0))
                    controls.throttle = throttle_cmd
                    if throttle_cmd > 0.9 and ball_forward_speed > 1400:
                        controls.boost = True
                    else:
                        controls.boost = False
        
        print(self.current, " ", self.dribbleGoal, " behindDist: ", behindDist)
                
        return controls
    
    def flick(self, car, ball, goal):
        to_goal = (goal - ball).flat().normalized()

        yaw = car.physics.rotation.yaw
        forward = Vec3(math.cos(yaw), math.sin(yaw), 0)

        angle_error = math.atan2(to_goal.y, to_goal.x) - math.atan2(forward.y, forward.x)
        if angle_error > math.pi:   angle_error -= 2*math.pi
        if angle_error < -math.pi:  angle_error += 2*math.pi

        flick_pitch = math.cos(angle_error)
        flick_roll = math.sin(angle_error)
        
        self.active_sequence = Sequence([
            ControlStep(0.05, SimpleControllerState(jump=True)),
            ControlStep(0.05, SimpleControllerState(jump=False)),
            ControlStep(0.2, SimpleControllerState(jump=True, pitch=-flick_pitch, yaw=flick_roll)),
            ControlStep(0.8, SimpleControllerState()),
        ])

    def kickoff(self, packet, loc):
        print("kickoff loc: ", loc)
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
    def behind_ball(self, team, loc, ball_loc, spc = False):
        y = loc.y
        ball_y = ball_loc.y
        if team == 1:
            y = loc.y * -1
            ball_y = ball_loc.y * -1
        if spc:
            return ball_y-y
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
        # self.renderer.draw_string_3d(my_pos, 8,8, f"Angle: {(angle_between*180/math.pi):.1f}", self.renderer.white())
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