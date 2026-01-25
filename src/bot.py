import math
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

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

        if self.behind_ball(self.team, my_location, ball_location):
            # target_location = ball_location
            target_location = self.adjust_target(self.team, ball_location)
            current = "ball"
        else:
            target_location = Vec3(0, 5120 * (-1 if self.team == 0 else 1), 100)
            current = "rotate"

        self.renderer.draw_line_3d(my_location, target_location, self.renderer.cyan())

        goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
        self.renderer.draw_line_3d(ball_location, goal, self.renderer.red())

        controls.steer = steer_toward_target(my_car, target_location)
        controls.throttle = 1

        if -0.5 < controls.steer < 0.5:
            controls.boost = True

        if current == "ball":
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
        elif current == "rotate":
            if my_location.dist(target_location) > 2000 and my_speed.length() < 2000 and my_speed.length() > 500 and abs(controls.steer) < 0.3:
                controls.boost = False
                self.front_flip(packet)


        return controls
    

    # Am I behind the ball?
    def behind_ball(self, team, loc, ball_loc):
        print
        if team == 1:
            y = loc.y * -1
            ball_y = ball_loc.y * -1
        if y < ball_y:
            return True
        return False

    def adjust_target(self, team, ball_loc):
        goal = Vec3(0, 5120 * (1 if self.team == 0 else -1), 0)
        direction = ball_loc - goal
        angle = math.atan2(direction.y, direction.x)
        # Offset parallel to the shot direction
        offset = Vec3(100 * math.cos(angle), 100 * math.sin(angle), 0)
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