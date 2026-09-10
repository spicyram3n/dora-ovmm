#!/usr/bin/env python3

import pathlib
import socket
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class GatewayProbe(Node):
    def __init__(self):
        super().__init__("vision_transport_executable_probe")
        self.received = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.qos = qos
        self.publisher = self.create_publisher(Image, "/vision_transport/e2e/input", qos)
        self.subscription = None

    def enable_output_subscription(self):
        self.subscription = self.create_subscription(
            Image, "/vision_transport/e2e/output", self._on_image, self.qos
        )

    def _on_image(self, image):
        self.received = image


def make_config(directory: pathlib.Path):
    tx = directory / "tx.yaml"
    rx = directory / "rx.yaml"
    tx_zenoh = directory / "zenoh_tx.json5"
    rx_zenoh = directory / "zenoh_rx.json5"
    zenoh_port = find_free_tcp_port()
    common = """streams:
  rgb:
    type: image
    input_topic: /vision_transport/e2e/input
    output_topic: /vision_transport/e2e/output
    zenoh_key: vision_transport/e2e/rgb
    max_rate_hz: 0.0
    queue_depth: 10
    codec:
      type: png
      png_level: 1
"""
    tx.write_text(common, encoding="utf-8")
    rx.write_text(
        """streams:
  rgb:
    type: image
    output_topic: /vision_transport/e2e/output
    zenoh_key: vision_transport/e2e/rgb
    queue_depth: 10
""",
        encoding="utf-8",
    )
    tx_zenoh.write_text(
        f"""{{
  mode: "peer",
  listen: {{ endpoints: ["tcp/127.0.0.1:{zenoh_port}"] }},
  scouting: {{ multicast: {{ enabled: false }} }},
  transport: {{ shared_memory: {{ enabled: false }} }},
}}
""",
        encoding="utf-8",
    )
    rx_zenoh.write_text(
        f"""{{
  mode: "peer",
  connect: {{ endpoints: ["tcp/127.0.0.1:{zenoh_port}"] }},
  scouting: {{ multicast: {{ enabled: false }} }},
  transport: {{ shared_memory: {{ enabled: false }} }},
}}
""",
        encoding="utf-8",
    )
    return tx, rx, tx_zenoh, rx_zenoh


def find_free_tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def make_image():
    image = Image()
    image.header.frame_id = "e2e_camera"
    image.width = 2
    image.height = 2
    image.encoding = "rgb8"
    image.is_bigendian = False
    image.step = 6
    image.data = bytes(
        [
            255,
            0,
            0,
            0,
            255,
            0,
            0,
            0,
            255,
            255,
            255,
            255,
        ]
    )
    return image


def launch_gateway(executable, config, zenoh_config):
    return subprocess.Popen(
        [
            executable,
            "--ros-args",
            "-p",
            f"config_path:={config}",
            "-p",
            f"zenoh_config_path:={zenoh_config}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_process(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()


def wait_for_process_start(processes, timeout_sec=3.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for process in processes:
            if process.poll() is not None:
                raise RuntimeError(f"process exited early with code {process.returncode}")
        time.sleep(0.05)


def spin_until(node, predicate, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
        time.sleep(0.05)
    return False


def main():
    if len(sys.argv) != 3:
        raise RuntimeError("usage: executable_gateway_test.py TX_EXECUTABLE RX_EXECUTABLE")

    tx_executable = sys.argv[1]
    rx_executable = sys.argv[2]

    with tempfile.TemporaryDirectory(prefix="vision_transport_e2e_") as temp:
        tx_config, rx_config, tx_zenoh_config, rx_zenoh_config = make_config(pathlib.Path(temp))
        tx_process = launch_gateway(tx_executable, tx_config, tx_zenoh_config)
        rx_process = launch_gateway(rx_executable, rx_config, rx_zenoh_config)
        processes = [tx_process, rx_process]

        try:
            wait_for_process_start(processes)
            rclpy.init(args=None)
            node = GatewayProbe()
            expected = make_image()

            no_upstream_subscription = spin_until(
                node, lambda: node.publisher.get_subscription_count() == 0, 2.0
            )
            assert no_upstream_subscription

            node.enable_output_subscription()
            has_upstream_subscription = spin_until(
                node, lambda: node.publisher.get_subscription_count() > 0, 6.0
            )
            assert has_upstream_subscription

            deadline = time.monotonic() + 8.0
            while node.received is None and time.monotonic() < deadline:
                node.publisher.publish(expected)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.05)

            if node.received is None:
                raise AssertionError("timed out waiting for reconstructed image")

            assert node.received.header.frame_id == expected.header.frame_id
            assert node.received.encoding == expected.encoding
            assert node.received.width == expected.width
            assert node.received.height == expected.height
            assert node.received.step == expected.step
            assert bytes(node.received.data) == bytes(expected.data)

            node.received = None
            stop_process(tx_process)
            spin_until(node, lambda: node.publisher.get_subscription_count() == 0, 2.0)

            tx_process = launch_gateway(tx_executable, tx_config, tx_zenoh_config)
            processes[0] = tx_process
            wait_for_process_start([tx_process])
            has_upstream_after_restart = spin_until(
                node, lambda: node.publisher.get_subscription_count() > 0, 8.0
            )
            assert has_upstream_after_restart

            deadline = time.monotonic() + 8.0
            while node.received is None and time.monotonic() < deadline:
                node.publisher.publish(expected)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.05)

            if node.received is None:
                raise AssertionError("timed out waiting for reconstructed image after TX restart")

            assert node.received.header.frame_id == expected.header.frame_id
            assert node.received.encoding == expected.encoding
            assert node.received.width == expected.width
            assert node.received.height == expected.height
            assert node.received.step == expected.step
            assert bytes(node.received.data) == bytes(expected.data)

            node.destroy_node()
            rclpy.shutdown()
        finally:
            for process in processes:
                stop_process(process)
            for process in processes:
                output = process.stdout.read() if process.stdout else ""
                if process.returncode not in (0, -15):
                    sys.stderr.write(output)


if __name__ == "__main__":
    main()
