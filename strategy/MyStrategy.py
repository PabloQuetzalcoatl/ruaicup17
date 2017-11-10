from model.ActionType import ActionType
from model.Game import Game
from model.Move import Move
from model.Player import Player
from model.World import World

from model.VehicleType import VehicleType

from collections import deque, namedtuple
import numpy as np
from math import *

LOG = True

TILE_SIZE = 32

UNIT_RADIUS = 2
SPACE = 1


POSITIONS = [(34.5, 34.5),  (93.5, 34.5),  (152.5, 34.5),
             (34.5, 93.5),  (93.5, 93.5),  (152.5, 93.5),
             (34.5, 152.5), (93.5, 152.5), (152.5, 152.5)]


class tile2d:
    def __init__(self, X, Y):
        self.X = X
        self.Y = Y

    def __str__(self):
        return 't2d(' + str(self.X) + ':' + str(self.Y) + ')'

    def __eq__(self, other):
        return ((self.X == other.X)and(self.Y == other.Y))

    def __gt__(self, other):
        return math.hypot(self.X, self.Y) > math.hypot(other.X, other.Y)


class point2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return 'p2d(' + str(self.x) + ':' + str(self.y) + ')'

    def __eq__(self, other):
        return ((self.x == other.x)and(self.y == other.y))

    def __gt__(self, other):
        return math.hypot(self.x, self.y) > math.hypot(other.x, other.y)

    def get_distance_to(self, x, y):
        return hypot(x - self.x, y - self.y)

    def get_distance_to_unit(self, unit):
        return self.get_distance_to(unit.x, unit.y)

    def get_distance_to_point(self, point):
        return self.get_distance_to(point.x, point.y)
    


def convert_tile2phys(inpt):
    """return center of tile in phis coord."""
    r_X = (inpt.X + 0.5) * TILE_SIZE
    r_Y = (inpt.Y + 0.5) * TILE_SIZE
    return point2d(r_X, r_Y)


def convert_phys2tile(inpt):
    """return tile in placed input phis point."""
    r = tile2d(int(inpt.x / TILE_SIZE), int(inpt.y / TILE_SIZE))
    return r


def rad2deg(a):
    return a * 180 / pi



def key_sort_by_dist(tupl):
        return tupl[2]

def print_map(w, name):
    print('  **********  ' + name + '  ************************  ')
    for xt in w:
        print(xt)  # - eto stolbets!!!! a ne stroka
    print('  ********** END   ' + name + '   *********************  ')


def print_vehicle(ve, name):
    print(' ++++++++++++ ' + name + ' ++++++++++++++++++')
    for v in ve.keys():
        vehicle = ve[v]
        print('{:4d} : id={:4d} player={} type={} x={} y={}'.format(
            v, vehicle.id, vehicle.player_id, vehicle.type, vehicle.x, vehicle.y))
        #print('{} : type={} model={}'.format(v, type( vehicle), vehicle))
    print(' ++++++++++++ END ' + name + ' ++++++++++++++++++')

# wrapper


class Ownership:
    ANY = 0
    ALLY = 1
    ENEMY = 2


def repr_v_type(vt):
    res = 'UNK'
    if vt == VehicleType.ARRV:
        res = 'ARRV'
    if vt == VehicleType.FIGHTER:
        res = 'FIGHTER'
    if vt == VehicleType.HELICOPTER:
        res = 'HELICOPTER'
    if vt == VehicleType.IFV:
        res = 'IFV'
    if vt == VehicleType.TANK:
        res = 'TANK'
    return res


def upd_vehicle(vehicle, vehicle_up):
    if vehicle.id != vehicle_up.id:
        raise ValueError('Vehicle ID mismatch [actual=%s, expected=%s].' % (
            vehicle_up.id, vehicle.id))
    vehicle.x = vehicle_up.x
    vehicle.y = vehicle_up.y
    vehicle.durability = vehicle_up.durability
    vehicle.remaining_attack_cooldown_ticks = vehicle_up.remaining_attack_cooldown_ticks
    vehicle.selected = vehicle_up.selected
    vehicle.groups = vehicle_up.groups
    
def ltrb_dict(l,t,r,b):
    r_dict={}
    r_dict['left']   = l
    r_dict['top']    = t
    r_dict['right']  = r
    r_dict['bottom'] = b
    return r_dict
    

class MyStrategy:
    terrain_map = []
    weather_map = []
    stored_move = None
    vehicleById = {}
    updateTickByVehicleId = {}
    delayed_moves = deque()

    def get_vehicles(self, ownership=Ownership.ANY, vehicle_type=None):
        vehicles = self.vehicleById.values()
        if ownership == Ownership.ALLY:
            vehicles = [v for v in vehicles if v.player_id == self.me.id]
        if ownership == Ownership.ENEMY:
            vehicles = [v for v in vehicles if v.player_id != self.me.id]
        if vehicle_type != None:
            vehicles = [v for v in vehicles if v.type == vehicle_type]

        return vehicles

    def battle_report(self):
        print('============ ALLY ====== ENEMY ============')
        for vehicle_type in range(5):
            vehicles = self.get_vehicles(
                Ownership.ALLY, vehicle_type=vehicle_type)
            enemy_vehicles = self.get_vehicles(
                Ownership.ENEMY, vehicle_type=vehicle_type)
            print('{:10s} :  {:3d}         {:3d} '.format(
                repr_v_type(vehicle_type), len(vehicles), len(enemy_vehicles)))

    def update_state(self, me, world, game, move):
        if world.tick_index == 0:
            self.initializeStrategy(world, game)
            self.initializeTick(me, world, game, move)
        else:
            self.initializeTick(me, world, game, move)

    def initializeStrategy(self, world, game):
        self.terrain_map = world.terrain_by_cell_x_y
        self.weather_map = world.weather_by_cell_x_y

    def initializeTick(self, me, world, game, move):
        self.me = me
        self.world = world
        self.game = game
        self.stored_move = move

        print('   new_v:' + str(len(world.new_vehicles)))
        print(' total_v:' + str(len(self.vehicleById)))
        print('up_new_v:' + str(len(world.vehicle_updates)))
        for vehicle in world.new_vehicles:
            self.vehicleById.update({vehicle.id: vehicle})
            self.updateTickByVehicleId.update({vehicle.id: world.tick_index})

        for vehicleUpdate in world.vehicle_updates:
            vehicleId = vehicleUpdate.id
            if (vehicleUpdate.durability == 0):
                self.vehicleById.pop(vehicleId)
                self.updateTickByVehicleId.pop(vehicleId)
            else:
                updated_vehicle = self.vehicleById.get(vehicleId)
                upd_vehicle(updated_vehicle, vehicleUpdate)
                self.vehicleById.update({vehicleId: updated_vehicle})
                self.updateTickByVehicleId.update(
                    {vehicleId: world.tick_index})

    def execute_delayed_move(self):
        if not self.delayed_moves:
            return False
        move_dict = self.delayed_moves.popleft()

        print(move_dict)
        for key in move_dict:
            setattr(self.stored_move, key, move_dict[key])

        return True

    def move(self, me,  world, game, move):
        print('--------{}------------'.format(world.tick_index))

        if LOG:
            f = open('..\\draw.txt', 'w')
        
        if LOG:
            self.battle_report()


        # === INITIALIZATION  ===========
        self.update_state(me, world, game, move)

        if self.me.remaining_action_cooldown_ticks > 0:
            return
        if self.execute_delayed_move():
            return
        self._move()
        self.execute_delayed_move()
        #=====================================
        if LOG:
            f.write('setColor 50 50 50')
            f.write('\n')

            for x in range(len(self.terrain_map)):
                for y in range(len(self.terrain_map[0])):
                    color = 'setColor 250 250 250'
                    terr_type = self.terrain_map[x][y]
                    if terr_type == 1:
                        color = 'setColor 250 0 0'
                    elif terr_type == 2:
                        color = 'setColor 0 250  0'
                    f.write(color)
                    f.write('\n')
                    c = convert_tile2phys(tile2d(x, y))
                    f.write('drawCircle ' + str(c.x) + ' ' +
                            str(c.y) + ' ' + str(int(TILE_SIZE / 2)))
                    f.write('\n')
            f.close()

    def _move(self):
        
        print('_move')
        #TEST
        if self.world.tick_index == 0:
            self.move_selection_rect(self.start_formation_selection(point2d(152.5, 34.5)))
            move_x = 100
            move_y = 0#to_p.y - from_p.y
            self.move_move(move_x, move_y, None)
            move_x = 0
            move_y = 100#to_p.y - from_p.y
            self.move_move(move_x, move_y, None)
        return

    
        #self.fighter_covered_tank()
        if self.world.tick_index == 0:
            # formation
##            self.cover(VehicleType.TANK, VehicleType.FIGHTER)
##            self.cover(VehicleType.IFV, VehicleType.HELICOPTER)
            move_list = self.info()
            self.take_initial_position(move_list)
##            for vt in [0,3,4]:
##              self.compact_square_formation(vt)
        else:
            pass
            #ATTACk
            #self.attack()
            
    def start_formation_selection(self, center_formation):
        l = center_formation.x - 25
        r = center_formation.x + 25
        t = center_formation.y - 25
        b = center_formation.y + 25
        return ltrb_dict(l,t,r,b)#{'left':l,'top':t, 'right':r, 'bottom':b} # refact to ltrb_dict


    
    def move_selection_rect(self, ltrb_dict):
        move_dict = {}
        move_dict['action'] = ActionType.CLEAR_AND_SELECT
        move_dict['left']   = ltrb_dict['left']
        move_dict['top']    = ltrb_dict['top']
        move_dict['right']  = ltrb_dict['right']
        move_dict['bottom'] = ltrb_dict['bottom']
        self.delayed_moves.append(move_dict)

    def move_move(self,x,y,max_speed):
        move_dict = {}
        move_dict['action'] = ActionType.MOVE
        move_dict['x']      = x
        move_dict['y']      = y
        if max_speed:
          move_dict['max_speed'] = max_speed
            
        self.delayed_moves.append(move_dict)
        
        
    def info(self):
        START_POINTS = [point2d(34.5, 34.5),  point2d(93.5, 34.5),  point2d(152.5, 34.5)]
        vt_list_1 = [0, 3, 4]
        vt_list_2 = list(vt_list_1)
        from_to_dist_list=[]
        # te kto i tak na svoem meste
        for vt in vt_list_1:
            vs = self.get_vehicles(Ownership.ALLY, vt)
            if vs:
               x = np.mean([v.x for v in vs])
               y = np.mean([v.y for v in vs])
            vehicle_p = point2d(x,y)

            if vehicle_p in START_POINTS:
                min_p = vehicle_p
                min_dist = 0
                from_to_dist_list.append( (vehicle_p, min_p, min_dist, vt) )
                START_POINTS.remove(min_p)
                vt_list_2.remove(vt)
        # do kogo blije
        for vt in vt_list_2:
            vs = self.get_vehicles(Ownership.ALLY, vt)
            if vs:
               x = np.mean([v.x for v in vs])
               y = np.mean([v.y for v in vs])
            vehicle_p = point2d(x,y)                
            min_dist = 1500
            min_p = START_POINTS[0]
            for p in START_POINTS:
                dist_p_to_vehicle = p.get_distance_to_point(vehicle_p)
                if dist_p_to_vehicle < min_dist:
                    min_dist = dist_p_to_vehicle
                    min_p = p
            from_to_dist_list.append( (vehicle_p, min_p, min_dist, vt) )
            START_POINTS.remove(min_p)    
            print('my {} center = {}'.format(repr_v_type(vt),(x, y)))
            print('nearest point = {}'.format(min_p))

        for x in from_to_dist_list:
            print('from {} to {} dist {} type {}'.format(x[0], x[1], x[2], repr_v_type(x[3])))
        move_list = sorted(from_to_dist_list, key=key_sort_by_dist)
        print('sorted____')
        for x in move_list:
            print('from {} to {} dist {} type {}'.format(x[0], x[1], x[2], repr_v_type(x[3])))
        #now move it
        return move_list
            
    def take_initial_position(self, move_list):
        for m in move_list:
            from_p = m[0]
            to_p = m[1]
            dist = m[2]
            if from_p != to_p :
                #select
                print('start select '+str(from_p))
                print('selection '+str(self.start_formation_selection(from_p)))
                self.move_selection_rect(self.start_formation_selection(from_p))
                if to_p.x == from_p.x:
                    #srazu vverx
                    move_x = 0
                    move_y = to_p.y - from_p.y
                    self.move_move(move_x, move_y, None)
                    print('srazu vverx '+str((move_x,move_y)))
                else:
                    # snachala vbok
                    move_x = to_p.x - from_p.x
                    move_y = 0#to_p.y - from_p.y
                    self.move_move(move_x, move_y, None)
                    print('vbok '+str((move_x,move_y)))
##                    #potom vverh
##                    move_x = 0
##                    move_y = to_p.y - from_p.y
##                    self.move_move(move_x, move_y, None)
##                    print('potom vverx '+str((move_x,move_y)))
                pass
        pass
            
    def attack(self):
        # --------- TANK -----------
        vs = self.get_vehicles(Ownership.ALLY, VehicleType.TANK)
        if vs:
           at_x = np.mean([v.x for v in vs])
           at_y = np.mean([v.y for v in vs])
           print('my tank center = {}'.format((at_x, at_y)))
        vs = self.get_vehicles(Ownership.ENEMY, VehicleType.TANK)
        if vs:
           et_x = np.mean([v.x for v in vs])
           et_y = np.mean([v.y for v in vs])
           print('enemy tank center = {}'.format((et_x, et_y)))
        target_x = et_x-at_x
        target_y = et_y-at_y
        print('target TANK = '+str((target_x,target_y)))
        self.delayed_moves.append(dict(
            action=ActionType.CLEAR_AND_SELECT,
            right=self.world.width,
            bottom=self.world.height,
            
        ))
        self.delayed_moves.append(dict(
            action=ActionType.MOVE,
            max_speed=0.4*0.6,
            x=target_x,
            y=target_y,
        ))
    def assembly(self):
        total_vs = []
        vs = self.get_vehicles(Ownership.ALLY, VehicleType.ARRV)
        total_vs +=vs
        if vs:
           at_a_x = np.mean([v.x for v in vs])
           at_a_y = np.mean([v.y for v in vs])
           a_a = (at_a_x,at_a_y)
        vs = self.get_vehicles(Ownership.ALLY, VehicleType.TANK)
        total_vs +=vs
        if vs:
           at_t_x = np.mean([v.x for v in vs])
           at_t_y = np.mean([v.y for v in vs])
           a_t = (at_t_x,at_t_y)
        vs = self.get_vehicles(Ownership.ALLY, VehicleType.IFV)
        total_vs +=vs
        if vs:
           at_i_x = np.mean([v.x for v in vs])
           at_i_y = np.mean([v.y for v in vs])
           a_i = (at_i_x,at_i_y)

        p0=()
        

           
        if total_vs:
           at_x = np.mean([v.x for v in total_vs])
           at_y = np.mean([v.y for v in total_vs])
           
        #VehicleType.ARRV
        target_x = at_x-at_a_x
        target_y = at_y-at_a_y
        self.delayed_moves.append(dict(
            action=ActionType.CLEAR_AND_SELECT,
            right=self.world.width,
            bottom=self.world.height,
            vehicle_type = VehicleType.ARRV
        ))
        # type2 goto type1
        self.delayed_moves.append(dict(
            action=ActionType.MOVE,
            #max_speed=0.4,
            x=target_x,
            y=target_y,
        ))
        #VehicleType.TANK
        target_x = at_x-at_t_x
        target_y = at_y-at_t_y
        self.delayed_moves.append(dict(
            action=ActionType.CLEAR_AND_SELECT,
            right=self.world.width,
            bottom=self.world.height,
            vehicle_type = VehicleType.TANK
        ))
        # type2 goto type1
        self.delayed_moves.append(dict(
            action=ActionType.MOVE,
            #max_speed=0.4,
            x=target_x,
            y=target_y,
        ))
        #VehicleType.IFV
        target_x = at_x-at_i_x
        target_y = at_y-at_i_y
        self.delayed_moves.append(dict(
            action=ActionType.CLEAR_AND_SELECT,
            right=self.world.width,
            bottom=self.world.height,
            vehicle_type = VehicleType.IFV
        ))
        # type2 goto type1
        self.delayed_moves.append(dict(
            action=ActionType.MOVE,
            #max_speed=0.4,
            x=target_x,
            y=target_y,
        ))        
        pass
    def cover(self,v_type1, v_type2):
        # --------- v_type1 -----------
        vs = self.get_vehicles(Ownership.ALLY, v_type1)
        if vs:
           at1_x = np.mean([v.x for v in vs])
           at1_y = np.mean([v.y for v in vs])
        # --------- v_type2 -----------
        vs = self.get_vehicles(Ownership.ALLY, v_type2)
        if vs:
           at2_x = np.mean([v.x for v in vs])
           at2_y = np.mean([v.y for v in vs])
        target_x = at1_x-at2_x
        target_y = at1_y-at2_y
        #select 2 
        self.delayed_moves.append(dict(
            action=ActionType.CLEAR_AND_SELECT,
            right=self.world.width,
            bottom=self.world.height,
            vehicle_type = v_type2
        ))
        # type2 goto type1
        self.delayed_moves.append(dict(
            action=ActionType.MOVE,
            #max_speed=0.4,
            x=target_x,
            y=target_y,
        ))
           
    def compact_square_formation(self,vehicle_type):
        # --------- FIGHTER -----------
        vs = self.get_vehicles(Ownership.ALLY, vehicle_type)
        if vs:
           a_x = np.mean([v.x for v in vs])
           a_y = np.mean([v.y for v in vs])
           min_a_x = min([v.x for v in vs])
           min_a_y = min([v.y for v in vs])
           max_a_x = max([v.x for v in vs])
           max_a_y = max([v.y for v in vs])

           # cycle col selection
           sign=1
           for i in range(10):
               left   = max_a_x-UNIT_RADIUS - i*(SPACE+2*UNIT_RADIUS)
               top    = min_a_y-UNIT_RADIUS 
               right  = max_a_x+UNIT_RADIUS - i*(SPACE+2*UNIT_RADIUS)
               bottom = max_a_y+UNIT_RADIUS

               self.move_selection_rect(ltrb_dict(left,top,right,bottom))
               #max_speed=0.4,
               x=1*i+1
               y=sign*1.5
               self.move_move(x,y,None)
               sign=-1*sign
        

        
print('Hello codewars v 0.0.5 ( start position maneures )')

print( bool(None))
# v 0.0.1 ( basics )
# v 0.0.2 ( get any type vehicles )
# v 0.0.3 ( add deque )
# v 0.0.4 ( first move )
# v 0.0.5 ( start position maneures )
#https://github.com/xmanatee/raic.2017/blob/master/MyStrategy.py
