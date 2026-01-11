import asyncio
import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmdvel_web_bridge")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("max_linear", 0.25)   # m/s
        self.declare_parameter("max_angular", 0.9)   # rad/s
        self.declare_parameter("timeout_s", 0.3)     # s, Deadman

        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        self.pub = self.create_publisher(Twist, topic, 10)

        self._last_rx = time.monotonic()
        self._v = 0.0
        self._w = 0.0

        self.create_timer(0.05, self._timer_cb)  # 20 Hz

    def update(self, v: float, w: float):
        v = max(-self.max_linear, min(self.max_linear, v))
        w = max(-self.max_angular, min(self.max_angular, w))
        self._v = v
        self._w = w
        self._last_rx = time.monotonic()

    def _timer_cb(self):
        dt = time.monotonic() - self._last_rx
        msg = Twist()
        if dt > self.timeout_s:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = float(self._v)
            msg.angular.z = float(self._w)
        self.pub.publish(msg)


async def ros_spin(node: Node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        await asyncio.sleep(0.001)


async def main_async():
    rclpy.init()
    bridge = CmdVelBridge()

    app = FastAPI()
    app.mount("/static", StaticFiles(directory="static"), name="static")

    @app.get("/")
    def index():
        return FileResponse("static/index.html")

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        try:
            while True:
                data = await ws.receive_text()
                obj = json.loads(data)
                bridge.update(float(obj.get("v", 0.0)), float(obj.get("w", 0.0)))
                await ws.send_text('{"ok": true}')
        except (WebSocketDisconnect, json.JSONDecodeError, ValueError):
            return

    import uvicorn
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="info")
    server = uvicorn.Server(config)

    await asyncio.gather(server.serve(), ros_spin(bridge))

    bridge.destroy_node()
    rclpy.shutdown()


def main():
    asyncio.run(main_async())
