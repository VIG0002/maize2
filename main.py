from enum import IntEnum
from typing import List
from ev3dev2.motor import MoveDifferential, SpeedPercent, SpeedRPM, Motor, LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import Sensor, TouchSensor, UltrasonicSensor, ColorSensor

US = UltrasonicSensor()
DRIVE = MoveDifferential()
SPEED = 20
DISTANCE = 290

class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class Node:
    def get_neighbour(self, direction: Direction) -> 'Node':
        ...

def look_around() -> List[Direction]:
    # This function will return a list of open directions
    # For now, we will return all directions
    return [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]

def move_forward():
    DRIVE.on_for_distance(SPEED, DISTANCE) # For now

def move_backward():
    DRIVE.on_for_distance(SPEED * -1, DISTANCE) # For now

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