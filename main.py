#!/usr/bin/env python3

"""
Here is exactly how to add that calibration logic to your existing functions:

## 1. Add a Global Heading Variable
At the top of your script (near your other global variables like visited), add a variable to track which absolute direction the robot chassis is currently facing:

# Initialize the robot facing absolute NORTH at the start of the maze
current_heading = Direction.NORTH  

## 2. Calibrate look_around()
Right now, look_around() assumes its relative scans match the map. You need to calibrate the relative sensor direction (d_rel) using the current_heading to find the absolute map direction (d_abs):

def look_around():
    directions = []

    for d_rel in [
        Direction.NORTH,
        Direction.EAST,
        Direction.WEST,
    ]:
        us_turn_to(d_rel)
        if (US.distance_centimeters * 10) > MAX_WALL_DETECTION_DISTANCE:
            # Calibrate: Add current heading to the relative sensor angle
            # Modulo 360 keeps the degree value within the Direction enum bounds (0, 90, 180, 270)
            d_abs = Direction((current_heading + d_rel) % 360)
            directions.append(d_abs)

    return directions

## 3. Calibrate move_to()
When dfs() passes an absolute map direction to move_to(direction), the robot needs to calculate how to move relative to its current physical heading.
You can find the relative movement required by subtracting current_heading from the target direction. Then, update the global heading afterward:

def move_to(direction):
    global current_heading
    
    # Calculate the required turn relative to where the robot is facing
    rel_move = Direction((direction - current_heading + 360) % 360)

    if rel_move == Direction.NORTH:
        move_forward()
    elif rel_move == Direction.SOUTH:
        move_backward()
    elif rel_move == Direction.WEST:
        move_backward()
        turn_anticlockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED, 90 - TURNING_DEGREES)
    elif rel_move == Direction.EAST:
        move_backward()
        turn_clockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED, 90 - TURNING_DEGREES)

    # Calibrate: Update our absolute heading to match the new tile's orientation
    current_heading = direction

## 4. Calibrate move_back()
When backing out of a node during the DFS backtrack phase, you must reverse this logic so the global compass heading steps backward correctly:

def move_back(last_move):
    global current_heading
    
    # Calculate the original relative movement that got us here
    # (Reconstructs what 'rel_move' was inside move_to)
    rel_move = Direction((last_move - current_heading + 360) % 360)

    if rel_move != Direction.NORTH:
        move_forward()
    if rel_move == Direction.WEST:
        turn_clockwise()
    elif rel_move == Direction.EAST:
        turn_anticlockwise()
    if rel_move != Direction.SOUTH:
        move_backward()
    if rel_move == Direction.WEST or rel_move == Direction.EAST:
        DRIVE.turn_degrees(SPEED, 90 - TURNING_DEGREES)

    # Calibrate: Revert the absolute heading back to the previous tile's heading
    # To step back, we subtract the change or compute it via the parent node's expected orientation
    current_heading = Direction((last_move + 180) % 360) 

"""

from enum import IntEnum, Enum
from ev3dev2.motor import MoveDifferential, SpeedPercent, Motor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.wheel import EV3Tire
from ev3dev2.sensor import INPUT_3, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
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

# Copied from source
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

# MAZE:
TILE_WIDTH = 290
VICTIM_WIDTH = 50

TILE_HALF_WIDTH = TILE_WIDTH / 2

# ROBOT: all coordinates are relative to the robot origin
SPEED = Speed.SLOW

ROBOT_WIDTH = 135
ROBOT_HEIGHT = 182

LEFT_WHEEL_PIN = OUTPUT_A
RIGHT_WHEEL_PIN = OUTPUT_B
WHEEL_TYPE = EV3Tire
WHEEL_MIDPOINT_GAP = 98 # measured: 88
WHEEL_MIDPOINT_GAP_MIDPOINT = Point(0, -28)
WHEEL_POLARITY = Motor.POLARITY_NORMAL

US_PIN = INPUT_3
US = UltrasonicSensor(US_PIN)
US_MOTOR_PIN = OUTPUT_C
US_MOTOR = MediumMotor(US_MOTOR_PIN)
US_MOTOR_MIDPOINT = Point(0, -52)
US_MOTOR_POLARITY = Motor.POLARITY_NORMAL
US_MOTOR_SPEED = Speed.SLOW
US_REL_DIRECTION = Direction.NORTH
US_NINETY_DEGREES = 97

CS_PIN = INPUT_4
CS = ColorSensor(CS_PIN)
CS_MIDPOINT = Point(0, 0) # technically 'ctrpoint'

MAX_WALL_DETECTION_DISTANCE = TILE_WIDTH / 2

REFLECTED_LIGHT_THRESHOLD = 75

DRIVE = MoveDifferential(LEFT_WHEEL_PIN, RIGHT_WHEEL_PIN, WHEEL_TYPE, WHEEL_MIDPOINT_GAP, WHEEL_POLARITY)
TURNING_DEGREES = 45
TIMESLEEP = 200/1000

visited = set()
harmed_victim_number = 0
unharmed_victim_number = 0
last_tile_was_start = None

leds = Leds()
sound = Sound()

class Node:
    def __init__(self):
        self.neighbours = {}
        self.tile_type = None

    def get_neighbour(self, direction):
        return self.neighbours[direction]

    def set_neighbour(self, direction, neighbour):
        self.neighbours[direction] = neighbour

def tile_type():
    color_value = Color(CS.color)       
    if CS.reflected_light_intensity >= REFLECTED_LIGHT_THRESHOLD:
        return TileType.START
    elif color_value == Color.WHITE:
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
        # move from the left/right, to the center (north)
        US_MOTOR.on_for_degrees(SpeedPercent((1 if US_REL_DIRECTION == Direction.WEST else -1) * US_MOTOR_SPEED), degrees=US_NINETY_DEGREES)

    US_REL_DIRECTION = direction
    
def look_around():
    directions = []

    for direction in [
        Direction.NORTH,
        Direction.EAST,
        Direction.WEST,
    ]:
        us_turn_to(direction)
        if (US.distance_centimeters * 10) > MAX_WALL_DETECTION_DISTANCE:
            directions.append(direction)

    return directions

def move_forward():
    global last_tile_was_start
    if tile_type() == TileType.START:
        DRIVE.on_for_distance(SPEED, (TILE_WIDTH - (ROBOT_HEIGHT - TILE_WIDTH / 2)))
        last_tile_was_start = True
    else:
        DRIVE.on_for_distance(SPEED, TILE_WIDTH)
        last_tile_was_start = False

def move_backward():
    if last_tile_was_start == True:
        DRIVE.on_for_distance(SPEED * -1, (TILE_WIDTH - (ROBOT_HEIGHT - TILE_WIDTH / 2)))
    else:
        DRIVE.on_for_distance(SPEED * -1, TILE_WIDTH) 

def turn_anticlockwise():
    DRIVE.turn_degrees(SPEED * -1, TURNING_DEGREES) # For now

def turn_clockwise():
    DRIVE.turn_degrees(SPEED, TURNING_DEGREES) # For now
        
def move_to(direction):
    if direction == Direction.NORTH:
        move_forward()
    elif direction == Direction.SOUTH:
        move_backward()
    elif direction == Direction.WEST:
        move_backward()
        turn_anticlockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED, 90 - TURNING_DEGREES)
    else:
        move_backward()
        turn_clockwise()
        move_forward()
        DRIVE.turn_degrees(SPEED, 90 - TURNING_DEGREES)

def move_back(last_move):
    if last_move != Direction.NORTH:
        move_forward()
    if last_move == Direction.WEST:
        turn_clockwise()
    elif last_move == Direction.EAST:
        turn_anticlockwise()
    if last_move != Direction.SOUTH:
        move_backward()
    if last_move == Direction.WEST or last_move == Direction.EAST:
        DRIVE.turn_degrees(SPEED, 90 - TURNING_DEGREES) # Turn straight after moving back to the previous node

def dfs(node):
    global visited, harmed_victim_number, unharmed_victim_number
    visited.add(node)
    open_directions = look_around()
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
                move_back(d)
                continue

            if neighbour.tile_type == TileType.HARMED_VICTIM:
                harmed_victim_number += 1
                sound.speak("Red")
                leds.set_color("LEFT", "RED")
                leds.set_color("RIGHT", "RED")
                time.sleep(1)
                leds.reset()

            elif neighbour.tile_type == TileType.UNHARMED_VICTIM:
                unharmed_victim_number += 1
                sound.speak("Green")
                leds.set_color("LEFT", "GREEN")
                leds.set_color("RIGHT", "GREEN")
                time.sleep(1)
                leds.reset()

            dfs(neighbour)
            move_back(d)

if __name__ == "__main__":
    start = Node()
    start.tile_type = TileType.START
    dfs(start) 
    sound.speak("Done")
    leds.animate_rainbow() 