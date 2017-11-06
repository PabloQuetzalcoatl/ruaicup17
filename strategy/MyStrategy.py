from model.ActionType import ActionType
from model.Game import Game
from model.Move import Move
from model.Player import Player
from model.World import World


class MyStrategy:
    def move(self, me: Player, world: World, game: Game, move: Move):
        print('--------{}------------'.format(world.tick_index))
        print(move.left)
        print(move.top)
        print(move.right)
        print(move.bottom)
        print(move.x)
        print(move.y)
        print(move.vehicle_type)
        if world.tick_index == 0:
            move.action = ActionType.CLEAR_AND_SELECT
            move.right = world.width
            move.bottom = world.height

        if world.tick_index == 1:
            move.action = ActionType.MOVE
            move.x = world.width
            move.y = world.height / 2.0
