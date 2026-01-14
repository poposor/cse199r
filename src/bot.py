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

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        my_car = packet.game_cars[self.index]
        controls = SimpleControllerState()

        field = self.get_field_info()

        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        my_location = Vec3(my_car.physics.location)
        ball_location = Vec3(packet.game_ball.physics.location)

        target_location = ball_location

        controls.steer = steer_toward_target(my_car, target_location)
        controls.throttle = 1

        if -0.5 < controls.steer < 0.5:
            controls.boost = True

        if my_location.dist(target_location) < 550:
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

        return controls
    
    def front_flip(self, packet):

        self.active_sequence = Sequence([
            ControlStep(0.05, SimpleControllerState(jump=True)),
            ControlStep(0.05, SimpleControllerState(jump=False)),
            ControlStep(0.2, SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(0.8, SimpleControllerState()),
        ])


        return self.active_sequence.tick(packet)