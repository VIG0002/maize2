#!/usr/bin/env python3
from enum import IntEnum, Enum
from typing import List, Union
from ev3dev2.motor import MoveDifferential, SpeedPercent, SpeedRPM, Motor, LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.wheel import EV3Tire, Wheel
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import Sensor, TouchSensor, UltrasonicSensor, ColorSensor
from shapely.geometry import Point  # pyright: ignore[reportMissingModuleSource]
import time
import math

Numeric = Union[int, float] # type hint any number

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

# ROBOT:
# all coordinates are relative to the robot origin
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
US_DISTANCE = US.distance_centimeters * 10
US_NINETY_DEGREES = 97

CS_PIN = INPUT_4
CS_MIDPOINT = Point(0, 0) # technically 'ctrpoint'

LEFT_TS_PIN = INPUT_1
LEFT_TS_ENDPOINT = Point(- ROBOT_WIDTH / 2, - ROBOT_HEIGHT + 28)
RIGHT_TS_PIN = INPUT_2
RIGHT_TS_ENDPOINT = Point(ROBOT_WIDTH / 2, - ROBOT_HEIGHT + 28)

DISTANCE_BETWEEN_ORIGIN_AND_TURNING_ORIGIN = CS_MIDPOINT.distance(WHEEL_MIDPOINT_GAP_MIDPOINT)
MAX_WALL_DETECTION_DISTANCE = TILE_WIDTH / 2

REFLECTED_LIGHT_THRESHOLD = 75
DRIVE = MoveDifferential(LEFT_WHEEL_PIN, RIGHT_WHEEL_PIN, WHEEL_TYPE, WHEEL_MIDPOINT_GAP, WHEEL_POLARITY)

def wait():
    """Sleep a short amount of time between movements, to prevent jamming"""
    time.sleep(400 / 1000)

class Node:
    def get_neighbour(self, direction: Direction) -> 'Node':
        ...

def us_turn_to(direction: Direction):
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
    
def is_open():
    return US_DISTANCE > MAX_WALL_DETECTION_DISTANCE

def look_around() -> List[Direction]:
    directions = []
    
    def add_if_open(direction: Direction):
        us_turn_to(direction)
        if is_open():
            directions.append(direction)

    return directions

def move_forward():
    DRIVE.on_for_distance(SPEED, MAX_WALL_DETECTION_DISTANCE) # For now

def move_backward():
    DRIVE.on_for_distance(SPEED * -1, MAX_WALL_DETECTION_DISTANCE) # For now

def turn_anticlockwise():
    DRIVE.turn_degrees(SPEED * -1, 45) # For now

def turn_clockwise():
    DRIVE.turn_degrees(SPEED, 45) # For now

def move_to(direction: Direction):
    move_backward()
    if direction == Direction.WEST:
        turn_anticlockwise()
    elif direction == Direction.EAST:
        turn_clockwise()
    move_forward()
    DRIVE.turn_degrees(SPEED, 90) # Turn straight after moving to the new node
    last_move = direction

def move_back(last_move: Direction):
    move_forward()
    if last_move == Direction.WEST:
        turn_clockwise()
    elif last_move == Direction.EAST:
        turn_anticlockwise()
    move_backward()

def dfs(node: Node):
    visited = set()
    visited.add(node)
    open_directions = look_around()
    for d in open_directions:
        neighbour = node.get_neighbour(d)
        if neighbour not in visited:
            visited.add(neighbour)
            move_to(neighbour)
            dfs(neighbour)
            move_back()

if __name__ == "__main__":
    dfs(Node()) 