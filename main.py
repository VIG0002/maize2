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
MAX_WALL_DETECTION_DISTANCE = TILE_WIDTH / 2

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
        self.tile_type = None

    def get_neighbour(self, direction):
        '''Return the neighbour node in the given direction.'''
        return self.neighbours[direction]

    def set_neighbour(self, direction, neighbour):
        '''Set the neighbour node in the given direction.'''
        self.neighbours[direction] = neighbour

def tile_type():
    '''Return the type of tile the robot is currently on, based on the colour sensor reading.'''
    color_value = Color(CS.color)       
    if color_value == Color.WHITE:
        return TileType.NORMAL
    elif color_value == Color.BLACK:
        return TileType.NOGO
    elif color_value == Color.RED:
        return TileType.HARMED_VICTIM
    elif color_value == Color.GREEN:
        return TileType.UNHARMED_VICTIM
    else:
        return TileType.NORMAL # default to normal if unknown color

def us_turn_to(direction):
    '''Turn the ultrasonic sensor to face the given direction.'''
    global US_REL_DIRECTION
    assert direction != Direction.SOUTH
    assert US_REL_DIRECTION != Direction.SOUTH

    if US_REL_DIRECTION == direction:
        return

    if direction in [Direction.EAST, Direction.WEST]:
        US_MOTOR.on(SpeedPercent((-1 if direction == Direction.WEST else 1) * US_MOTOR_SPEED))
        US_MOTOR.wait_until_not_moving()
        US_MOTOR.off()

    elif direction == Direction.NORTH:
        US_MOTOR.on_for_degrees(SpeedPercent((1 if US_REL_DIRECTION == Direction.WEST else -1) * US_MOTOR_SPEED), degrees=US_NINETY_DEGREES)

    US_REL_DIRECTION = direction
    
def look_around():
    '''Return a list of directions that are open (no wall) from the current position.'''
    global current_heading
    open_global_directions = []

    for rel_dir in [
        Direction.NORTH,
        Direction.EAST,
        Direction.WEST,
    ]:
        us_turn_to(rel_dir)
        if (US.distance_centimeters * 10) > MAX_WALL_DETECTION_DISTANCE:
            global_dir = Direction((current_heading + rel_dir) % 360)
            open_global_directions.append(global_dir)

    us_turn_to(Direction.NORTH) # Reset ultrasonic sensor to face north after looking around
    return open_global_directions

def move_forward():
    global last_tile_was_start
    touching = bool(ts_map[Direction.WEST].is_pressed) and bool(ts_map[Direction.EAST].is_pressed)
    if touching:
        DRIVE.on_for_distance(SPEED, (TILE_WIDTH - (ROBOT_HEIGHT - TILE_WIDTH / 2)))
        last_tile_was_start = True
    else:
        last_tile_was_start = False
        DRIVE.on_for_distance(SPEED, (TILE_WIDTH / 2 + 15))
        if tile_type() != TileType.NOGO:
            DRIVE.on_for_distance(SPEED, (TILE_WIDTH / 2 - 15))
        else:
            move_backward()

def move_backward():
    if last_tile_was_start == True:
        DRIVE.on_for_distance(SPEED * -1, (TILE_WIDTH - (ROBOT_HEIGHT - TILE_WIDTH / 2)))
    else:
        DRIVE.on_for_distance(SPEED * -1, TILE_WIDTH) 

def turn_anticlockwise():
    DRIVE.turn_degrees(SPEED * -1, TURNING_DEGREES)

def turn_clockwise():
    DRIVE.turn_degrees(SPEED, TURNING_DEGREES)
        
def move_to(direction):
    '''Move the robot to the neighbouring node in the given direction. '''
    global current_heading

    turn_offset = (direction - current_heading) % 360
    if turn_offset == 0:
        move_forward()
    elif turn_offset == 180:
        move_backward()
    elif turn_offset == 270:
        move_backward()
        turn_anticlockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED * -1, (US_NINETY_DEGREES - TURNING_DEGREES))
    elif turn_offset == 90:
        move_backward()
        turn_clockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED, (US_NINETY_DEGREES - TURNING_DEGREES))

    current_heading = direction

def move_back(last_move):
    '''Reverse the last move made by the robot, returning it to the previous node.'''
    global current_heading

    if last_move == Direction.NORTH:
        move_backward()
    elif last_move == Direction.SOUTH:
        move_forward()
    elif last_move == Direction.WEST:
        move_backward()
        turn_clockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED, (US_NINETY_DEGREES - TURNING_DEGREES))
    elif last_move == Direction.EAST:
        move_backward()
        turn_anticlockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED * -1, (US_NINETY_DEGREES - TURNING_DEGREES))

    current_heading = last_move

def dispense_rescue_kit():
    '''Dispense a rescue kit for a harmed victim.'''
    DISPENSER_MOTOR.on_for_degrees(SpeedPercent(DISPENSER_MOTOR_SPEED), 360)

def dfs(node):
    '''Depth-first search algorithm to explore the maze. The robot will visit each node, check for victims, and keep track of visited nodes. It will also dispense rescue kits for harmed victims and keep count of harmed and unharmed victims.'''
    global visited, harmed_victim_number, unharmed_victim_number
    visited.add(node)
    open_directions = look_around()
    us_turn_to(Direction.NORTH)

    if not open_directions:
        return

    explored_child = False
    for d in open_directions:
        if d not in node.neighbours:
            neighbour = Node()
            node.set_neighbour(d, neighbour)
            neighbour.set_neighbour((Direction((d + 180) % 360)), node)
        else:
            neighbour = node.get_neighbour(d)

        if neighbour not in visited:
            move_to(d)

            neighbour.tile_type = tile_type()

            if neighbour.tile_type == TileType.NOGO:
                visited.add(neighbour)
                move_back(d)
                us_turn_to(Direction.NORTH)
                continue

            explored_child = True

            if neighbour.tile_type == TileType.HARMED_VICTIM:
                harmed_victim_number += 1
                dispense_rescue_kit()
                sound.speak("Red", volume=100)
                leds.set_color("LEFT", "RED")
                leds.set_color("RIGHT", "RED")
                time.sleep(1)
                leds.reset()

            elif neighbour.tile_type == TileType.UNHARMED_VICTIM:
                unharmed_victim_number += 1
                sound.speak("Green", volume=100)
                leds.set_color("LEFT", "GREEN")
                leds.set_color("RIGHT", "GREEN")
                time.sleep(1)
                leds.reset()

            dfs(neighbour)
            move_back(d)
            us_turn_to(Direction.NORTH)

    if not explored_child:
        return
    
if __name__ == "__main__":
    us_turn_to(Direction.NORTH)
    start = Node()
    start.tile_type = TileType.START
    dfs(start)
    sound.speak("Done", volume=100)
    leds.animate_rainbow()