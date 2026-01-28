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

    def initialize_agent(self):
        print("Knarr is on team: ", self.team)

    current = "ball"
    isKickoff = False
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
        ball_location = Vec3(packet.game_ball.physics.location)
        if packet.game_info.is_kickoff_pause == False:
            self.isKickoff = False
        if packet.game_info.is_kickoff_pause and self.isKickoff == False:
            self.isKickoff = True
            print("begin kickoff?")
            self.current = "kickoff"
        elif self.behind_ball(self.team, my_location, ball_location):
            # target_location = ball_location
            target_location = self.adjust_target(my_car, self.team, ball_location)
            self.current = "ball"
            if my_location.dist(ball_location) > 1500:
                ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
                ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + 1)


                if ball_in_future is not None:
                    target_location = Vec3(ball_in_future.physics.location)
        else:
            target_location = Vec3(0, 5120 * (-1 if self.team == 0 else 1), 100)
            self.current = "rotate"
        
        if self.current != "kickoff":
            self.renderer.draw_line_3d(my_location, target_location, self.renderer.cyan())

            goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
            self.renderer.draw_line_3d(ball_location, goal, self.renderer.red())

            controls.steer = steer_toward_target(my_car, target_location)
            controls.throttle = 1
        steer_mag = abs(controls.steer)
        if steer_mag < 0.5:
            controls.boost = True
        else:
            min_thresh = 0.8
            if steer_mag > min_thresh:
                prob = (steer_mag - 0.7) / 1
                if random.random() < prob:
                    controls.handbrake = True
                    # print(f"Partial power slide (p={prob:.2f}):", controls.steer)

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
            if my_location.dist(target_location) < 550 and abs(controls.steer) < 0.5:
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


        return controls
    
    def kickoff(self, packet, loc):
        if loc == 0:
            self.active_sequence = Sequence([
                ControlStep(0.5, SimpleControllerState(steer=1, throttle=1)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=-1, throttle=1, jump=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False)),
                ControlStep(0.05, SimpleControllerState(pitch=-1, yaw=-1, throttle=1, jump=True)),
                ControlStep(0.05, SimpleControllerState(throttle=1, jump=False)),
            ])
        elif loc < 5:
            return self.active_sequence.tick(packet)
        else:
            self.current = "ball"


    # Am I behind the ball?
    def behind_ball(self, team, loc, ball_loc):
        print
        if team == 1:
            y = loc.y * -1
            ball_y = ball_loc.y * -1
        if y < ball_y:
            return True
        return False

    def adjust_target(self, my_car, team, ball_loc):
        goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
        direction = ball_loc - goal
        angle = math.atan2(direction.y, direction.x)
        # Offset parallel to the shot direction
        offset = Vec3((my_car.hitbox.width/2 + 91.25) * math.cos(angle), 91.25 * math.sin(angle), 0)
        self.renderer.draw_line_3d(ball_loc + offset, ball_loc, self.renderer.red())
        return ball_loc + offset

    def front_flip(self, packet):

        self.active_sequence = Sequence([
            ControlStep(0.05, SimpleControllerState(jump=True)),
            ControlStep(0.05, SimpleControllerState(jump=False)),
            ControlStep(0.2, SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(0.8, SimpleControllerState()),
        ])


        return self.active_sequence.tick(packet)