#!/usr/bin/env python3

import rclpy
from fastapi import FastAPI, APIRouter
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from modules.platform.platform_api import router as platform_router
from modules.arm.arm_api import router as arm_router
from modules.video.video_api import router as video_router
from modules.autonomy.autonomy_api import router as autonomy_router
from modules.science.science_api import router as science_router


#            _____ _____
#      /\   |  __ \_   _|
#     /  \  | |__) || |
#    / /\ \ |  ___/ | |
#   / ____ \| |    _| |_
#  /_/    \_\_|   |_____|

if __name__ == "__main__":
    rclpy.init(args=None)
    node = rclpy.create_node("groundstation_api")

    router = APIRouter(prefix="/station/system/rover")
    router.include_router(platform_router)
    router.include_router(arm_router)
    router.include_router(video_router)
    router.include_router(autonomy_router)
    router.include_router(science_router)

    app = FastAPI()
    app.add_middleware(
        CORSMiddleware,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
        allow_origins=["*"],
    )
    app.include_router(router)

    config = uvicorn.Config(app, host="0.0.0.0", port=8000, debug=True)
    server = uvicorn.Server(config)
    server.run()
