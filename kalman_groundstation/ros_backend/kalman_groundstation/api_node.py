#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fastapi import FastAPI, APIRouter
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
# from kalman_groundstation.modules.platform.platform_api import router as platform_router
# from kalman_groundstation.modules.arm.arm_api import ArmAPI  ## TODO: implement when we have more info
from kalman_groundstation.modules.arm.arm_api import ArmAPI
from kalman_groundstation.modules.platform.platform_api import PlatformAPI


from kalman_groundstation.modules.radio.radio_api import RadioAPI

from kalman_groundstation.modules.video.video_api import VideoAPI
# from kalman_groundstation.modules.video.video_api import router as video_router

from kalman_groundstation.modules.autonomy.autonomy_api import AutonomyAPI
# from kalman_groundstation.modules.autonomy.autonomy_api import router as autonomy_router


from kalman_groundstation.modules.science.science_api import ScienceAPI
# from kalman_groundstation.modules.science.science_api import router as science_router


#            _____ _____
#      /\   |  __ \_   _|
#     /  \  | |__) || |
#    / /\ \ |  ___/ | |
#   / ____ \| |    _| |_
#  /_/    \_\_|   |_____|


class GroundstationAPI(Node):
    def __init__(self):
        super().__init__(node_name='groundstation_api')
        self.router = APIRouter(prefix="/station/system/rover")
        # self.router.include_router(platform_router)

        # self.arm_router = arm_router()
        # self.arm_api = ArmAPI(self, self.router)
        self.router.include_router(ArmAPI(self))
        self.router.include_router(PlatformAPI(self))
        self.router.include_router(RadioAPI(self))
        self.router.include_router(VideoAPI(self))
        self.router.include_router(AutonomyAPI(self))
        self.router.include_router(ScienceAPI(self))

        
    

# if __name__ == "__main__":
def main():
    rclpy.init(args=None)
    # node = rclpy.create_node("groundstation_api")
    node = GroundstationAPI()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    # router = APIRouter(prefix="/station/system/rover")
    # router.include_router(arm_router)
    # router.include_router(video_router)
    # router.include_router(autonomy_router)
    # router.include_router(science_router)
    app = FastAPI()
    app.add_middleware(
        CORSMiddleware,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
        allow_origins=["*"],
    )
    app.include_router(node.router)

    config = uvicorn.Config(app, host="0.0.0.0", port=8000)
    server = uvicorn.Server(config)
    server.run()
