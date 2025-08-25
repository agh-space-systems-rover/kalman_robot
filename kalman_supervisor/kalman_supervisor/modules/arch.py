import time
from cob_srvs.srv import SetInt
from rclpy.duration import Duration

from kalman_supervisor.module import Module


class Arch(Module):
    def __init__(self):
        super().__init__("arch")

    def configure(self) -> None:
        self.supervisor.declare_parameter("arch.num_cameras", 0)

    def activate(self) -> None:
        self.__clients = []
        for i in range(self.supervisor.get_parameter("arch.num_cameras").value):
            self.__clients.append(
                self.supervisor.create_client(SetInt, f"arch/take_photo{i}")
            )

    def deactivate(self) -> None:
        for client in self.__clients:
            self.supervisor.destroy_client(client)

    def take_photos(self, label: int) -> None:
        req = SetInt.Request(data=label)
        for client in self.__clients:
            client.call_async(req)
