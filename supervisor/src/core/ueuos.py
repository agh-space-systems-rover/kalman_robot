import rclpy
from rclpy import Node
from rclpy.client import Client
from enum import IntEnum
from ueuos.srv import State

client = Client()

class Ueuos:
    class Color(IntEnum):
        GREEN = 3
        BLUE = 2
        RED = 1

    def __init__(self):
        client.wait_for_service("/ueuos/set_state")
        self.__set_state_service = Node().create_client(State, "/ueuos/set_state")
        self.__color = self.Color.BLUE
        self.__set_color(self.Color.BLUE)

    def __set_color(self, color: Color):
        self.__set_state_service(color.value)
        self.__color = color

    @property
    def color(self) -> Color:
        return self.__color

    @color.setter
    def color(self, color: Color):
        self.__set_color(color)
