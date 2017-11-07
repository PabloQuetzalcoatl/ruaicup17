from model.ActionType import ActionType
from model.Game import Game
from model.Move import Move
from model.Player import Player
from model.World import World

from model.VehicleType import VehicleType

LOG = True

TILE_SIZE = 32



class tile2d:
    def __init__(self, X, Y):
      self.X = X
      self.Y = Y
    def __str__(self):
     return "t2d("+str(self.X)+":"+str(self.Y)+")"
    def __eq__(self, other):
     return ( (self.X == other.X)and( self.Y == other.Y) )
    def __gt__(self, other):
     return math.hypot(self.X, self.Y) > math.hypot(other.X, other.Y)

class point2d:
    def __init__(self, x, y):
      self.x = x
      self.y = y
    def __str__(self):
     return "p2d("+str(self.x)+":"+str(self.y)+")"
    def __eq__(self, other):
     return ( (self.x == other.x)and( self.y == other.y) )
    def __gt__(self, other):
     return math.hypot(self.x, self.y) > math.hypot(other.x, other.y)
    def get_distance_to(self, x, y):
        return hypot(x - self.x, y - self.y)
    def get_distance_to_unit(self, unit):
        return self.get_distance_to(unit.x, unit.y)
    def get_distance_to_point(self, point):
        return self.get_distance_to(point.x, point.y)
    
def convert_tile2phys(inpt):

    '''

    return center of tile in phis coord

    '''

    r_X = (inpt.X + 0.5) * TILE_SIZE

    r_Y = (inpt.Y + 0.5) * TILE_SIZE

    return point2d(r_X, r_Y)



def convert_phys2tile(inpt):

    '''

    return tile in placed input phis point

    '''

    r  = tile2d( int(inpt.x/TILE_SIZE), int(inpt.y/TILE_SIZE) )

    return r



def rad2deg(a):

    return a*180/pi


def print_map(w, name):
  print( '  **********  '+ name +'  ************************  ')
  for xt in w:
    print( xt )# - eto stolbets!!!! a ne stroka
  print( '  ********** END   '+ name +'   *********************  ')

def print_vehicle(ve, name):
  print( ' ++++++++++++ '+ name +' ++++++++++++++++++')
  for v in ve.keys():
      vehicle = ve[v]
      print('{:4d} : id={:4d} player={} type={} x={} y={}'.format(v, vehicle.id, vehicle.player_id, vehicle.type, vehicle.x, vehicle.y))
      #print('{} : type={} model={}'.format(v, type( vehicle), vehicle))
  print( ' ++++++++++++ END '+ name +' ++++++++++++++++++')

# wrapper
def upd_vehicle(vehicle, vehicle_up):
        if vehicle.id != vehicle_up.id:
            raise ValueError("Vehicle ID mismatch [actual=%s, expected=%s]." % (vehicle_up.id, vehicle.id))

        vehicle.x = vehicle_up.x
        vehicle.y = vehicle_up.y
        vehicle.durability = vehicle_up.durability
        vehicle.remaining_attack_cooldown_ticks = vehicle_up.remaining_attack_cooldown_ticks
        vehicle.selected = vehicle_up.selected
        vehicle.groups = vehicle_up.groups    

def stream_vehicles(vehicles, owner=None, vehicle_type=None):
    if (owner!=None) and (vehicle_type!=None):
        result = dict((k, v) for (k, v) in vehicles.items() if  (v.player_id==owner)and(v.type==vehicle_type))
    elif (owner!=None) and (vehicle_type==None):
        result = dict((k, v) for (k, v) in vehicles.items() if  v.player_id==owner)
    elif (owner==None) and (vehicle_type!=None):
        result = dict((k, v) for (k, v) in vehicles.items() if  v.type==vehicle_type)
    else:
        result = vehicles
    return result        
    


class MyStrategy:
    terrain_map =[]
    weather_map =[]
##
##    me;
##    world;
##    game;
##    move;
##
    vehicleById           = {}
    updateTickByVehicleId = {}
##    delayedMoves          = []
    def initializeStrategy(self,world, game):
        self.terrain_map = world.terrain_by_cell_x_y;
        self.weather_map = world.weather_by_cell_x_y;

    def initializeTick(self, me, world, game, move): 
        self.me = me;
        self.world = world;
        self.game = game;


        print("new_v:"+str(len(world.new_vehicles)))
        print("me_v:"+str(len(self.vehicleById)))
        print("up_new_v:"+str(len(world.vehicle_updates)))
        for vehicle in world.new_vehicles:
            self.vehicleById.update( {vehicle.id : vehicle} )
            self.updateTickByVehicleId.update( {vehicle.id : world.tick_index} )
        

        for vehicleUpdate in world.vehicle_updates:
            vehicleId = vehicleUpdate.id
            #print('upd v id '+str(vehicleId))
            if (vehicleUpdate.durability == 0):
                self.vehicleById.pop(vehicleId);
                self.updateTickByVehicleId.pop(vehicleId);
            else:
                updated_vehicle = self.vehicleById.get(vehicleId)
               # print('type : '+str(type(updated_vehicle)))
               # print('updated : '+str(updated_vehicle))
                upd_vehicle(updated_vehicle, vehicleUpdate)
                self.vehicleById.update( {vehicleId : updated_vehicle} )
                self.updateTickByVehicleId.update( {vehicleId : world.tick_index} )

     
    def move(self, me: Player, world: World, game: Game, move: Move):
        if LOG:
            f = open('..\\draw.txt', 'w')
        
        print('--------{}------------'.format(world.tick_index))
        # === INITIALIZATION  ===========
        if world.tick_index == 0:
            self.initializeStrategy(world, game)
            self.initializeTick(me, world, game, move)
        else:
            self.initializeTick(me, world, game, move)




        if LOG:
            # world map

            #print_map(self.terrain_map, "terrain_map")
            #print_vehicle(self.vehicleById, 'VEHICLES by Id')
            # my tanks
            my_tanks = stream_vehicles(self.vehicleById, me.id, VehicleType.TANK)
            print_vehicle(my_tanks, 'my_tanks by Id')
            any_tanks = stream_vehicles(self.vehicleById, vehicle_type = VehicleType.TANK)
            print_vehicle(any_tanks, 'any_tanks by Id')
            f.write('setColor 50 50 50'  )
            f.write('\n')
            
            for x in range(len(self.terrain_map)):
                for y in range(len(self.terrain_map[0])):
                    color = 'setColor 250 250 250'
                    terr_type = self.terrain_map[x][y]
                    if terr_type==1:
                        color = 'setColor 250 0 0'
                    elif terr_type==2:
                        color = 'setColor 0 250  0'                          
                    f.write(color)
                    f.write('\n')
                    c=convert_tile2phys(tile2d(x, y))
                    f.write('drawCircle ' +str(c.x)+" "+str(c.y)+" "+str(int(TILE_SIZE/2)) )
                    f.write('\n')
            f.close()
            
print("Hello codewars v 0.0.1 ( basics)")
print(VehicleType.TANK)
