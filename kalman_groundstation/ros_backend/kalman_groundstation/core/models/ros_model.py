import json
from pydantic import BaseModel
from std_msgs.msg import String as Json
from typing import Any


class DictModel(BaseModel):
    data: dict[str, Any]


class RosModel(BaseModel):
    """
    Model for ROS messages. It is a wrapper around pydantic's BaseModel.
    """
    @classmethod
    def from_ros_msg(cls, msg: Json):
        return DictModel(data=json.loads(msg))

    def to_ros_msg(self):
        msg = Json()
        msg.data = self.model_dump_json()
        return msg
