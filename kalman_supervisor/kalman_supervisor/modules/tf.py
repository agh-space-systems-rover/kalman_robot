import numpy as np

from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformableObject

from kalman_supervisor.module import Module

def quat_to_basis(q: np.ndarray) -> np.ndarray:
    return (
        np.array([
            1 - 2 * (q[2] * q[2] + q[3] * q[3]),
            2 * (q[1] * q[2] + q[3] * q[0]),
            2 * (q[1] * q[3] - q[2] * q[0]),
            2 * (q[1] * q[2] - q[3] * q[0]),
            1 - 2 * (q[1] * q[1] + q[3] * q[3]),
            2 * (q[2] * q[3] + q[1] * q[0]),
            2 * (q[1] * q[3] + q[2] * q[0]),
            2 * (q[2] * q[3] - q[1] * q[0]),
            1 - 2 * (q[1] * q[1] + q[2] * q[2]),
        ]).reshape(3, 3).T
    )

class TF(Module):
    def __init__(self):
        super().__init__("tf")

    def activate(self) -> None:
        self.__buffer = Buffer()
        self.__listener = TransformListener(self.__buffer, self.supervisor)

    def deactivate(self) -> None:
        self.__listener.unregister()

    def can_transform(self, target_frame: str, source_frame: str, time: Time = Time()) -> bool:
        return self.__buffer.can_transform(target_frame, source_frame, time)

    def transform(self, object_stamped: TransformableObject, target_frame: str) -> TransformableObject:
        if object_stamped.header.frame_id == target_frame:
            return object_stamped
        return self.__buffer.transform(object_stamped, target_frame)

    def lookup_numpy(
        self, target_frame: str, source_frame: str, time: Time = Time()
    ) -> np.ndarray:
        if target_frame == source_frame:
            return np.eye(4)

        transform = self.__buffer.lookup_transform(target_frame, source_frame, time).transform

        pos = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        matrix = np.eye(4)
        matrix[:3, :3] = quat_to_basis([
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
        ])
        matrix[:3, 3] = pos
        return matrix
    
    def transform_numpy(
        self, pos: np.ndarray, target_frame: str, source_frame: str, time: Time = Time()
    ) -> np.ndarray:
        mat4x4 = self.lookup_numpy(target_frame, source_frame, time)

        if pos.shape == (3,):
            return (mat4x4 @ np.append(pos, 1))[:3]
        else:
            raise ValueError("pos must be a 3D numpy array.")

    def rotate_numpy(
        self, dir: np.ndarray, target_frame: str, source_frame: str, time: Time = Time()
    ) -> np.ndarray:
        mat4x4 = self.lookup_numpy(target_frame, source_frame, time)
        
        if dir.shape == (3,):
            return (mat4x4[:3, :3] @ dir)
        else:
            raise ValueError("dir must be a 3D numpy array.")
    