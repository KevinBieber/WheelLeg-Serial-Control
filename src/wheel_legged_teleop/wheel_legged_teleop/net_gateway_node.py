#!/usr/bin/env python3
"""网络网关：对接 pc_host 的 TCP 状态/指令端口，并桥接到 ROS 话题。

- 监听 command_port：每行 JSON → 队列 → 主线程发布 /wheel_legged/cmd
- 订阅 /wheel_legged/status → 推给已连接的 status 客户端（每行 JSON）
"""

from __future__ import annotations

import json
import queue
import socket
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NetGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("net_gateway")
        self.declare_parameter("status_bind", "0.0.0.0")
        self.declare_parameter("status_port", 8082)
        self.declare_parameter("command_bind", "0.0.0.0")
        self.declare_parameter("command_port", 8083)

        self.cmd_pub = self.create_publisher(String, "/wheel_legged/cmd", 10)
        self.create_subscription(String, "/wheel_legged/status", self._on_status, 10)

        self._status_clients: List[socket.socket] = []
        self._clients_lock = threading.Lock()
        # TCP 读线程只入队，由定时器在 executor 线程里 publish（rclpy 更稳）
        self._cmd_queue: queue.Queue[str] = queue.Queue(maxsize=200)
        self._cmd_rx_count = 0
        self._cmd_pub_count = 0

        status_bind = self.get_parameter("status_bind").get_parameter_value().string_value
        status_port = int(self.get_parameter("status_port").value)
        cmd_bind = self.get_parameter("command_bind").get_parameter_value().string_value
        cmd_port = int(self.get_parameter("command_port").value)

        self._status_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._status_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._status_server.bind((status_bind, status_port))
        self._status_server.listen(5)

        self._cmd_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._cmd_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._cmd_server.bind((cmd_bind, cmd_port))
        self._cmd_server.listen(5)

        threading.Thread(target=self._accept_status, daemon=True).start()
        threading.Thread(target=self._accept_cmd, daemon=True).start()
        self.create_timer(0.01, self._flush_cmd_queue)
        self.create_timer(2.0, self._log_stats)
        self.get_logger().info(f"status TCP :{status_port}  command TCP :{cmd_port}")

    def _flush_cmd_queue(self) -> None:
        while True:
            try:
                line = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            msg = String()
            msg.data = line
            self.cmd_pub.publish(msg)
            self._cmd_pub_count += 1

    def _log_stats(self) -> None:
        if self._cmd_rx_count or self._cmd_pub_count:
            self.get_logger().info(
                f"cmd stats: tcp_rx={self._cmd_rx_count} ros_pub={self._cmd_pub_count} "
                f"queue={self._cmd_queue.qsize()}"
            )

    def _accept_status(self) -> None:
        while rclpy.ok():
            try:
                conn, addr = self._status_server.accept()
            except OSError:
                break
            self.get_logger().info(f"status client {addr}")
            with self._clients_lock:
                self._status_clients.append(conn)

    def _accept_cmd(self) -> None:
        while rclpy.ok():
            try:
                conn, addr = self._cmd_server.accept()
            except OSError:
                break
            self.get_logger().info(f"command client {addr}")
            threading.Thread(target=self._cmd_reader, args=(conn, addr), daemon=True).start()

    def _cmd_reader(self, conn: socket.socket, addr) -> None:
        try:
            f = conn.makefile("r", encoding="utf-8", newline="\n")
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    json.loads(line)
                except json.JSONDecodeError:
                    self.get_logger().warn(f"bad JSON from {addr}: {line[:80]}")
                    continue
                self._cmd_rx_count += 1
                try:
                    self._cmd_queue.put_nowait(line)
                except queue.Full:
                    try:
                        self._cmd_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self._cmd_queue.put_nowait(line)
        except OSError:
            pass
        finally:
            self.get_logger().info(f"command client closed {addr}")
            try:
                conn.close()
            except OSError:
                pass

    def _on_status(self, msg: String) -> None:
        line = (msg.data.strip() + "\n").encode("utf-8")
        dead: List[socket.socket] = []
        with self._clients_lock:
            clients = list(self._status_clients)
        for c in clients:
            try:
                c.sendall(line)
            except OSError:
                dead.append(c)
        if dead:
            with self._clients_lock:
                self._status_clients = [c for c in self._status_clients if c not in dead]
            for c in dead:
                try:
                    c.close()
                except OSError:
                    pass


def main() -> None:
    rclpy.init()
    node = NetGatewayNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._status_server.close()
            node._cmd_server.close()
        except OSError:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
