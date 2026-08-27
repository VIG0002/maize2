from enum import IntEnum
from typing import List
from ev3dev2.motor import MoveDifferential, SpeedPercent, SpeedRPM, Motor, LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import Sensor, TouchSensor, UltrasonicSensor, ColorSensor

US = UltrasonicSensor()
DRIVE = MoveDifferential()

class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class Node:
    def get_neighbour(self, direction: Direction) -> 'Node':
        ...

def look_around() -> List[Direction]:
    ...

def move_to(neighbour: Node):
    DRIVE.on_for_distance() # For now

def move_back():
    ... # For now

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