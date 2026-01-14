from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

class Knarr(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)

    def initialize_agent(self):
        print("Knarr is on team: ", self.team)

    targetBoostPad = 3

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        my_car = packet.game_cars[self.index]
        controller = SimpleControllerState()

        field = self.get_field_info()
        target = field.boost_pads[self.targetBoostPad]

        my_location = my_car.physics.location
        target_location = target.location

        print("Car location: ", my_location)
        print("Target location: ", target_location)
        self.renderer.draw_string_3d(my_location, 1, 1, f'{my_location} : {target_location}', self.renderer.white())

        controller.throttle = 0.1

        return controller

    