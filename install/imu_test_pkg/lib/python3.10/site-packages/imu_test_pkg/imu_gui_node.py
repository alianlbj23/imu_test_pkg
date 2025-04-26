import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import pybullet as p
import pybullet_data
import time
import threading
from transforms3d.quaternions import quat2mat


class ImuVisualizer(Node):
    def __init__(self):
        super().__init__("imu_visualizer_orientation_axes")

        # 訂閱 IMU topic
        self.sub_imu = self.create_subscription(
            Imu, "/imu/data_raw", self.imu_callback, 10
        )

        # 儲存目前畫的線 ID
        self.marker_ids = []
        # 軸長度
        self.axis_length = 0.3
        # 將原點提升到 0.1m 以上，避免畫在地板上
        self.origin = np.array([0.0, 0.0, 0.1])

        # 啟動 PyBullet
        self.init_pybullet()

        # 啟動模擬背景 thread
        self._running = True
        sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        sim_thread.start()

    def init_pybullet(self):
        """初始化 PyBullet GUI 世界"""
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        # 使用我們自己的 stepSimulation
        # p.setRealTimeSimulation(1)
        p.loadURDF("plane.urdf")

        # 把鏡頭往上拉一點，方便看原點
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,
            cameraYaw=50,
            cameraPitch=-35,
            cameraTargetPosition=[0, 0, 0.1],
        )

    def _simulation_loop(self):
        """背景定時呼叫 stepSimulation()"""
        while self._running and rclpy.ok():
            p.stepSimulation()
            time.sleep(1.0 / 240.0)  # 240Hz 模擬

    def imu_callback(self, msg: Imu):
        # Debug 印一下確認有沒有進來
        self.get_logger().info(f"Got IMU orientation: {msg.orientation.w:.3f}")

        # 清除舊線
        for mid in self.marker_ids:
            p.removeUserDebugItem(mid)
        self.marker_ids.clear()

        # 取出四元數 (x, y, z, w)
        qx, qy, qz, qw = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        # transforms3d 要的順序 [w, x, y, z]
        rot_matrix = quat2mat([qw, qx, qy, qz])

        # 計算三條軸
        x_axis = rot_matrix @ np.array([1, 0, 0]) * self.axis_length
        y_axis = rot_matrix @ np.array([0, 1, 0]) * self.axis_length
        z_axis = rot_matrix @ np.array([0, 0, 1]) * self.axis_length

        # 紅色 X
        self.marker_ids.append(
            p.addUserDebugLine(
                self.origin.tolist(),
                (self.origin + x_axis).tolist(),
                [1, 0, 0],
                lineWidth=4,
            )
        )
        # 綠色 Y
        self.marker_ids.append(
            p.addUserDebugLine(
                self.origin.tolist(),
                (self.origin + y_axis).tolist(),
                [0, 1, 0],
                lineWidth=4,
            )
        )
        # 藍色 Z
        self.marker_ids.append(
            p.addUserDebugLine(
                self.origin.tolist(),
                (self.origin + z_axis).tolist(),
                [0, 0, 1],
                lineWidth=4,
            )
        )

    def destroy_node(self):
        # 停掉模擬 thread
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
