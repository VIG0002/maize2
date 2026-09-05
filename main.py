#!/usr/bin/env python3
from enum import IntEnum, Enum
from ev3dev2.motor import MoveDifferential, SpeedPercent, Motor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.wheel import EV3Tire
from ev3dev2.sensor import INPUT_3, INPUT_4, INPUT_1, INPUT_2
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor, TouchSensor
from shapely.geometry import Point
from ev3dev2.sound import Sound
from ev3dev2.led import Leds
import time

class Direction(IntEnum):
    """The four cardinal directions, whose values correspond to clockwise rotation relative to the positive y-axis"""
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270

class Speed(IntEnum):
    """Different motor speeds, to be used with SpeedPercent"""
    SLOW = 10
    MEDIUM = 20
    FAST = 30

class Color(IntEnum):
    """The different colour values returned by ev3"""
    NONE = 0
    BLACK = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4
    RED = 5
    WHITE = 6
    BROWN = 7

class TileType(Enum):
    """Types of tile surfaces.
    Start: silver reflective tile
    Normal: white tile
    Nogo: black tile
    Harmed victim: red tile
    Unharmed victim: green tile
    """
    START = 0
    NORMAL = 1
    NOGO = 2
    HARMED_VICTIM = 3
    UNHARMED_VICTIM = 4

## MAZE:
TILE_WIDTH = 290
VICTIM_WIDTH = 50
TILE_HALF_WIDTH = TILE_WIDTH / 2
MAX_WALL_DETECTION_DISTANCE = TILE_WIDTH

## ROBOT
SPEED = Speed.SLOW

ROBOT_WIDTH = 135
ROBOT_HEIGHT = 182

# WHEELS
LEFT_WHEEL_PIN = OUTPUT_A
RIGHT_WHEEL_PIN = OUTPUT_D
WHEEL_TYPE = EV3Tire
WHEEL_MIDPOINT_GAP = 98 # measured: 88
WHEEL_MIDPOINT_GAP_MIDPOINT = Point(0, -28)
WHEEL_POLARITY = Motor.POLARITY_NORMAL
DRIVE = MoveDifferential(LEFT_WHEEL_PIN, RIGHT_WHEEL_PIN, WHEEL_TYPE, WHEEL_MIDPOINT_GAP, WHEEL_POLARITY)
TURNING_DEGREES = 36 # NOTE: A bit off. I need to make it plus or minus 1 or 2 degrees. Experiment. 

# ULTRASONIC SENSOR
US_PIN = INPUT_3
US = UltrasonicSensor(US_PIN)
US_MOTOR_PIN = OUTPUT_B
US_MOTOR = MediumMotor(US_MOTOR_PIN)
US_MOTOR_MIDPOINT = Point(0, -52)
US_MOTOR_POLARITY = Motor.POLARITY_NORMAL
US_MOTOR_SPEED = Speed.SLOW
US_REL_DIRECTION = Direction.NORTH
US_NINETY_DEGREES = 97

# COLOUR SENSOR
CS_PIN = INPUT_4
CS = ColorSensor(CS_PIN)
CS.mode = 'COL-COLOR'

# TOUCH SENSOR
LEFT_TS_PIN = INPUT_1
RIGHT_TS_PIN = INPUT_2

# DISPENSER MOTOR
DISPENSER_MOTOR_PIN = OUTPUT_C
DISPENSER_MOTOR = MediumMotor(DISPENSER_MOTOR_PIN)
DISPENSER_MOTOR_SPEED = Speed.SLOW 

## GLOBAL VARIABLES
visited = set()
harmed_victim_number = 0
unharmed_victim_number = 0
last_tile_was_start = None
current_heading = Direction.NORTH
ts_map = {Direction.WEST: TouchSensor(LEFT_TS_PIN), Direction.EAST: TouchSensor(RIGHT_TS_PIN)}

leds = Leds()
sound = Sound()

class Node:
    def __init__(self):
        '''Initialize a new node with no neighbours and no tile type.'''
        self.neighbours = {}
        print('[Node.__init__] created empty neighbours dictionary')
        self.tile_type = None
        print('[Node.__init__] set tile_type to None')

    def get_neighbour(self, direction):
        '''Return the neighbour node in the given direction.'''
        print(f'[Node.get_neighbour] looking up direction={direction.name}')
        neighbour = self.neighbours[direction]
        print(f'[Node.get_neighbour] found neighbour id={id(neighbour)}')
        return neighbour

    def set_neighbour(self, direction, neighbour):
        '''Set the neighbour node in the given direction.'''
        self.neighbours[direction] = neighbour
        print(f'[Node.set_neighbour] direction={direction.name} neighbour id={id(neighbour)}')

def tile_type():
    '''Return the type of tile the robot is currently on, based on the colour sensor reading.'''
    print('[tile_type] reading colour sensor')
    color_value = Color(CS.color)
    print(f'[tile_type] colour={color_value.name}')
    if color_value == Color.WHITE:
        print('[tile_type] colour is WHITE; returning NORMAL')
        return TileType.NORMAL
    elif color_value == Color.BLACK:
        print('[tile_type] colour is BLACK; returning NOGO')
        return TileType.NOGO
    elif color_value == Color.RED:
        print('[tile_type] colour is RED; returning HARMED_VICTIM')
        return TileType.HARMED_VICTIM
    elif color_value == Color.GREEN:
        print('[tile_type] colour is GREEN; returning UNHARMED_VICTIM')
        return TileType.UNHARMED_VICTIM
    else:
        print('[tile_type] colour is unknown; returning NORMAL')
        return TileType.NORMAL # default to normal if unknown color

def us_turn_to(direction):
    '''Turn the ultrasonic sensor to face the given direction.'''
    global US_REL_DIRECTION
    print(f'[us_turn_to] requested direction={direction.name}')
    assert direction != Direction.SOUTH
    print('[us_turn_to] direction is not SOUTH')
    assert US_REL_DIRECTION != Direction.SOUTH
    print(f'[us_turn_to] current sensor direction={US_REL_DIRECTION.name} is valid')

    if US_REL_DIRECTION == direction:
        print('[us_turn_to] sensor is already facing requested direction; returning')
        return

    if direction in [Direction.EAST, Direction.WEST]:
        print('[us_turn_to] requested direction is EAST or WEST')
        US_MOTOR.on(SpeedPercent((-1 if direction == Direction.WEST else 1) * US_MOTOR_SPEED))
        print('[us_turn_to] started ultrasonic motor')
        US_MOTOR.wait_until_not_moving()
        print('[us_turn_to] ultrasonic motor stopped moving')
        US_MOTOR.off()
        print('[us_turn_to] switched ultrasonic motor off')

    elif direction == Direction.NORTH:
        print('[us_turn_to] requested direction is NORTH')
        US_MOTOR.on_for_degrees(SpeedPercent((1 if US_REL_DIRECTION == Direction.WEST else -1) * US_MOTOR_SPEED), degrees=US_NINETY_DEGREES)
        print('[us_turn_to] rotated ultrasonic motor to NORTH')

    US_REL_DIRECTION = direction
    print(f'[us_turn_to] updated sensor direction to {US_REL_DIRECTION.name}')
    
def look_around():
    '''Return a list of directions that are open (no wall) from the current position.'''
    global current_heading
    print(f'[look_around] starting with robot heading={current_heading.name}')
    open_global_directions = []
    print('[look_around] created empty open directions list')

    for rel_dir in [
        Direction.NORTH,
        Direction.EAST,
        Direction.WEST,
    ]:
        print(f'[look_around] checking relative direction={rel_dir.name}')
        us_turn_to(rel_dir)
        distance = US.distance_centimeters * 10
        print(f'[look_around] measured distance millimetres={distance}')
        is_open = distance > MAX_WALL_DETECTION_DISTANCE
        print(f'[look_around] is_open={is_open}')
        if is_open:
            global_dir = Direction((current_heading + rel_dir) % 360)
            print(f'[look_around] converted to global direction={global_dir.name}')
            open_global_directions.append(global_dir)
            print(f'[look_around] added {global_dir.name} to open directions')

    us_turn_to(Direction.NORTH) # Reset ultrasonic sensor to face north after looking around
    print(f'[look_around] finished; open directions={[direction.name for direction in open_global_directions]}')
    return open_global_directions

def move_forward():
    global last_tile_was_start
    print(f'[move_forward] starting; robot heading={current_heading.name}')
    touching = bool(ts_map[Direction.EAST].is_pressed) and bool(ts_map[Direction.WEST].is_pressed)
    print(f'[move_forward] touch sensors indicate touching={touching}')
    if touching:
        print('[move_forward] both touch sensors pressed; using start-tile distance')
        DRIVE.on_for_distance(SPEED, (TILE_WIDTH - (ROBOT_HEIGHT - TILE_WIDTH / 2)))
        print('[move_forward] completed start-tile drive')
        last_tile_was_start = True
        print('[move_forward] set last_tile_was_start=True')
    else:
        print('[move_forward] not on start tile; using normal-tile movement')
        last_tile_was_start = False
        print('[move_forward] set last_tile_was_start=False')
        DRIVE.on_for_distance(SPEED, ((TILE_WIDTH / 2) + 15))
        print('[move_forward] completed first half of normal-tile drive')
        if tile_type() != TileType.NOGO:
            print('[move_forward] destination is not NOGO; completing second half')
            DRIVE.on_for_distance(SPEED, ((TILE_WIDTH / 2) - 15))
            print('[move_forward] completed second half of normal-tile drive')
        else:
            print('[move_forward] destination is NOGO; stopped after first half')

def move_backward():
    print(f'[move_backward] starting; last_tile_was_start={last_tile_was_start}')
    if last_tile_was_start == True:
        print('[move_backward] reversing from start tile')
        DRIVE.on_for_distance((SPEED * -1), (TILE_WIDTH - (ROBOT_HEIGHT - TILE_WIDTH / 2)))
        print('[move_backward] completed start-tile reverse')
    else:
        print('[move_backward] reversing one tile')
        DRIVE.on_for_distance((SPEED * -1), TILE_WIDTH)
        print('[move_backward] completed one-tile reverse')

def turn_anticlockwise():
    print('[turn_anticlockwise] starting turn')
    DRIVE.turn_degrees(SPEED, TURNING_DEGREES * -1)
    print('[turn_anticlockwise] completed turn')

def turn_clockwise():
    print('[turn_clockwise] starting turn')
    DRIVE.turn_degrees(SPEED, TURNING_DEGREES)
    print('[turn_clockwise] completed turn')
        
def move_to(direction):
    '''Move the robot to the neighbouring node in the given direction. '''
    global current_heading
    print(f'[move_to] requested direction={direction.name}; current heading={current_heading.name}')

    turn_offset = (direction - current_heading) % 360
    print(f'[move_to] calculated turn offset={turn_offset}')
    if turn_offset == 0:
        print('[move_to] offset 0; moving forward')
        move_forward()
    elif turn_offset == 180:
        print('[move_to] offset 180; moving backward')
        move_backward()
    elif turn_offset == 270:
        print('[move_to] offset 270; moving backward, turning anticlockwise, then moving forward')
        move_backward()
        print('[move_to] backward movement complete for offset 270')
        turn_anticlockwise()
        print('[move_to] anticlockwise turn complete for offset 270')
        move_forward()
        print('[move_to] forward movement complete for offset 270')
        DRIVE.turn_degrees((SPEED * -1), (90 - TURNING_DEGREES))
        print('[move_to] completed final alignment turn for offset 270')
    elif turn_offset == 90:
        print('[move_to] offset 90; moving backward, turning clockwise, then moving forward')
        move_backward()
        print('[move_to] backward movement complete for offset 90')
        turn_clockwise()
        print('[move_to] clockwise turn complete for offset 90')
        move_forward()
        print('[move_to] forward movement complete for offset 90')
        DRIVE.turn_degrees(SPEED, (90 - TURNING_DEGREES))
        print('[move_to] completed final alignment turn for offset 90')

    current_heading = direction
    print(f'[move_to] updated robot heading={current_heading.name}')

def move_back(last_move):
    '''Reverse the last move made by the robot, returning it to the previous node.'''
    global current_heading
    print(f'[move_back] starting; last_move={last_move.name}; current heading={current_heading.name}')

    if last_move == Direction.NORTH:
        print('[move_back] last move NORTH; moving backward')
        move_backward()
    elif last_move == Direction.SOUTH:
        print('[move_back] last move SOUTH; moving forward')
        move_forward()
    elif last_move == Direction.WEST:
        print('[move_back] last move WEST; moving backward, turning clockwise, moving forward')
        move_backward()
        print('[move_back] backward movement complete for WEST')
        turn_clockwise()
        print('[move_back] clockwise turn complete for WEST')
        move_forward()
        print('[move_back] forward movement complete for WEST')
        DRIVE.turn_degrees(SPEED, (90 - TURNING_DEGREES))
        print('[move_back] final alignment turn complete for WEST')
    elif last_move == Direction.EAST:
        print('[move_back] last move EAST; moving backward, turning anticlockwise, moving forward')
        move_backward()
        print('[move_back] backward movement complete for EAST')
        turn_anticlockwise()
        print('[move_back] anticlockwise turn complete for EAST')
        move_forward()
        print('[move_back] forward movement complete for EAST')
        DRIVE.turn_degrees(SPEED * -1, (90 - TURNING_DEGREES))
        print('[move_back] final alignment turn complete for EAST')

    current_heading = last_move
    print(f'[move_back] updated robot heading={current_heading.name}')

def dispense_rescue_kit():
    '''Dispense a rescue kit for a harmed victim.'''
    DISPENSER_MOTOR.on_for_degrees(SpeedPercent(DISPENSER_MOTOR_SPEED), 360)
    print('[dispense_rescue_kit] completed dispenser motor action')

def dfs(node):
    '''Depth-first search algorithm to explore the maze. The robot will visit each node, check for victims, and keep track of visited nodes. It will also dispense rescue kits for harmed victims and keep count of harmed and unharmed victims.'''
    global visited, harmed_victim_number, unharmed_victim_number, current_heading
    print(f'[dfs] entering node id={id(node)} heading={current_heading.name}')
    visited.add(node)
    print(f'[dfs] added node to visited; visited count={len(visited)}')
    open_directions = look_around()
    print(f'[dfs] open directions={[direction.name for direction in open_directions]}')
    us_turn_to(Direction.NORTH)
    print('[dfs] reset ultrasonic sensor to NORTH')

    if not open_directions:
        print('[dfs] no open directions; returning')
        return

    explored_child = False
    print('[dfs] set explored_child=False')
    for d in open_directions:
        print(f'[dfs] beginning direction iteration d={d.name}')
        if d not in node.neighbours:
            print(f'[dfs] direction {d.name} has no neighbour; creating one')
            neighbour = Node()
            print(f'[dfs] created neighbour id={id(neighbour)}')
            node.set_neighbour(d, neighbour)
            print(f'[dfs] stored neighbour in direction {d.name}')
            neighbour.set_neighbour((Direction((d + 180) % 360)), node)
            print(f'[dfs] stored reverse neighbour direction={Direction((d + 180) % 360).name}')
        else:
            print(f'[dfs] direction {d.name} already has neighbour; retrieving it')
            neighbour = node.get_neighbour(d)
            print(f'[dfs] retrieved neighbour id={id(neighbour)}')

        if neighbour not in visited:
            print(f'[dfs] neighbour id={id(neighbour)} is unvisited')
            parent_heading = current_heading
            print(f'[dfs] saved parent heading={parent_heading.name}')
            move_to(d)
            print(f'[dfs] completed move_to({d.name})')

            neighbour.tile_type = tile_type()
            print(f'[dfs] recorded neighbour tile_type={neighbour.tile_type.name}')

            if neighbour.tile_type == TileType.NOGO:
                print('[dfs] neighbour is NOGO; marking visited and backing out')
                visited.add(neighbour)
                print(f'[dfs] marked NOGO neighbour visited; visited count={len(visited)}')
                move_back(d)
                print(f'[dfs] completed move_back({d.name}) for NOGO')
                current_heading = parent_heading
                print(f'[dfs] restored parent heading={current_heading.name} after NOGO')
                us_turn_to(Direction.NORTH)
                print('[dfs] reset ultrasonic sensor to NORTH after NOGO')
                continue

            explored_child = True
            print('[dfs] set explored_child=True')

            if neighbour.tile_type == TileType.HARMED_VICTIM:
                print('[dfs] found harmed victim')
                harmed_victim_number += 1
                print(f'[dfs] harmed victim count={harmed_victim_number}')
                dispense_rescue_kit()
                print('[dfs] rescue kit action complete')
                sound.speak("Red", volume=100)
                print('[dfs] spoke Red')
                leds.set_color("LEFT", "RED")
                print('[dfs] set LEFT LED RED')
                leds.set_color("RIGHT", "RED")
                print('[dfs] set RIGHT LED RED')
                time.sleep(1)
                print('[dfs] completed victim indication delay')
                leds.reset()
                print('[dfs] reset LEDs')

            elif neighbour.tile_type == TileType.UNHARMED_VICTIM:
                print('[dfs] found unharmed victim')
                unharmed_victim_number += 1
                print(f'[dfs] unharmed victim count={unharmed_victim_number}')
                sound.speak("Green", volume=100)
                print('[dfs] spoke Green')
                leds.set_color("LEFT", "GREEN")
                print('[dfs] set LEFT LED GREEN')
                leds.set_color("RIGHT", "GREEN")
                print('[dfs] set RIGHT LED GREEN')
                time.sleep(1)
                print('[dfs] completed victim indication delay')
                leds.reset()
                print('[dfs] reset LEDs')

            dfs(neighbour)
            print(f'[dfs] returned from child node id={id(neighbour)}')
            move_back(d)
            print(f'[dfs] completed move_back({d.name}) after child')
            current_heading = parent_heading
            print(f'[dfs] restored parent heading={current_heading.name} after child')
            us_turn_to(Direction.NORTH)
            print('[dfs] reset ultrasonic sensor to NORTH after child')
        else:
            print(f'[dfs] neighbour id={id(neighbour)} already visited; skipping')

    if not explored_child:
        print('[dfs] no child was explored; returning')
        return
    print('[dfs] finished all open directions')
    
if __name__ == "__main__":
    print('[main] program started')
    us_turn_to(Direction.EAST)
    print('[main] ultrasonic sensor moved to EAST')
    us_turn_to(Direction.NORTH)
    print('[main] ultrasonic sensor moved to NORTH')
    start = Node()
    print(f'[main] created start node id={id(start)}')
    start.tile_type = TileType.START
    print('[main] set start tile_type=START')
    dfs(start)
    print('[main] DFS completed')
    sound.speak("Done", volume=100)
    print('[main] spoke Done')
    leds.animate_rainbow()
    print('[main] started rainbow LED animation')