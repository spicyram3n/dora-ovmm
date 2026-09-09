"""Real ROS/ZeroMQ loopback test. Run in a sourced Humble environment."""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, CameraInfo

HERE = Path(__file__).resolve().parent


class GatewayTest(unittest.TestCase):
    def test_delivery_restart_and_source_stall(self):
        routes = json.loads((HERE / 'routes.json').read_text())
        routes['camera_info']['source'] = 'pc'  # Exercise the reverse direction too.
        for index, route in enumerate(routes.values()):
            route['port'] = 18601 + index
            route['input'] = '/gateway_test/source/stream_' + str(index)
            route['output'] = '/gateway_test/dest/stream_' + str(index)
        received = {}
        rclpy.init()
        node = Node('gateway_test')
        messages = [Image(height=480, width=640, encoding='bgr8', step=1920, data=bytes([100]*921600)),
                    Image(height=480, width=640, encoding='16UC1', step=1280, data=bytes([232,3]*307200)),
                    LaserScan(angle_min=-1., angle_max=1., ranges=[1.,2.,3.]),
                    CameraInfo(width=6, height=4)]
        pubs, subs = [], []
        for (name, route), message in zip(routes.items(), messages):
            pubs.append(node.create_publisher(type(message), route['input'], qos_profile_sensor_data))
            subs.append(node.create_subscription(type(message), route['output'],
                lambda m, name=name: received.__setitem__(name, m), qos_profile_sensor_data))
        def pump(seconds, publish=True):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if publish:
                    for pub, msg in zip(pubs, messages):
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.header.frame_id = 'test_optical'
                        pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.02)
        children = []
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'routes.json'
            config.write_text(json.dumps(routes))
            def start(role):
                p = subprocess.Popen([sys.executable, str(HERE/'gateway.py'), role,
                    '--bind','127.0.0.1','--peer','127.0.0.1','--config',str(config)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                children.append(p)
                return p
            try:
                robot, pc = start('robot'), start('pc')
                pump(8)
                self.assertEqual(set(received), set(routes))
                self.assertEqual(bytes(received['depth'].data), bytes(messages[1].data))
                self.assertEqual(list(received['scan'].ranges), [1.,2.,3.])
                self.assertEqual(received['rgb'].header.frame_id, 'test_optical')
                self.assertEqual(received['rgb'].encoding, 'bgr8')
                messages[1] = Image(height=4, width=6, encoding='32FC1', step=24,
                                    data=struct.pack('<6f', 0., 1.25, float('nan'), float('inf'), -1., 12.) * 4)
                robot.send_signal(signal.SIGINT)
                robot.wait(timeout=8)
                received.clear()
                robot = start('robot')
                pump(8)
                self.assertEqual(set(received), set(routes))
                self.assertEqual(received['depth'].encoding, '32FC1')
                self.assertEqual(bytes(received['depth'].data), bytes(messages[1].data))
                pump(2, publish=False)
                stamps = {k: (m.header.stamp.sec, m.header.stamp.nanosec) for k,m in received.items()}
                pump(2, publish=False)
                self.assertEqual(stamps, {k: (m.header.stamp.sec,m.header.stamp.nanosec) for k,m in received.items()})
            finally:
                for p in children:
                    if p.poll() is None:
                        p.send_signal(signal.SIGINT)
                for p in children:
                    try:
                        p.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        p.kill()
                        p.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    os.environ.setdefault('ROS_DOMAIN_ID', '73')
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    unittest.main()
