"""Explicit-peer latest-sample ROS gateway; one isolated process per route."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

LIMIT = 32 * 1024 * 1024


def validate(routes):
    ports = []
    outputs = []
    inputs = []
    for name, route in routes.items():
        if route['source'] not in ('robot', 'pc'):
            raise ValueError(f'{name}: source must be robot or pc')
        if route['codec'] not in ('cdr', 'zstd', 'jpeg'):
            raise ValueError(f'{name}: unsupported codec')
        if route['codec'] == 'jpeg' and route['type'] != 'sensor_msgs/msg/Image':
            raise ValueError('JPEG requires Image')
        if not 1024 <= route['port'] <= 65535 or not 0 < route['rate'] <= 120 or not 0 < route['max_age'] <= 60:
            raise ValueError(f'{name}: invalid port/rate/max_age')
        ports.append(route['port'])
        outputs.append(route['output'])
        inputs.append(route['input'])
    if len(set(ports)) != len(ports) or len(set(outputs)) != len(outputs) or set(inputs) & set(outputs):
        raise ValueError('Ports/outputs must be unique; outputs must not feed gateway inputs')


def worker(args, route):
    import zmq
    import zstandard as zstd
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message, deserialize_message
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(args=[])
    node = Node('gateway_' + args.role + '_' + args.stream)
    kind = get_message(route['type'])
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    sending = route['source'] == args.role
    latest = [None, 0.0, 0]
    lock = threading.Lock()
    def receive(message):
        with lock:
            latest[:] = [message, time.monotonic(), latest[2] + 1]
    if sending:
        sub = node.create_subscription(kind, route['input'], receive, qos)
        def source_status():
            with lock:
                _, arrival, sequence = latest
            age = time.monotonic() - arrival if arrival else float('inf')
            node.get_logger().info(f'source publishers={sub.get_publisher_count()}; received={sequence}; latest age={age:.2f}s')
        node.create_timer(5.0, source_status)
    else:
        pub = node.create_publisher(kind, route['output'], qos)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    context = zmq.Context()
    compressor = zstd.ZstdCompressor(level=1)
    decompressor = zstd.ZstdDecompressor()
    bridge = None
    if route['codec'] == 'jpeg':
        import cv2
        import numpy as np
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CompressedImage
        bridge = CvBridge()
        cv2.setNumThreads(1)

    def encode(message):
        if bridge:
            bgr = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise ValueError('JPEG encoding failed')
            wire = CompressedImage(header=message.header, format='jpeg', data=encoded.tobytes())
            return serialize_message(wire)
        raw = serialize_message(message)
        if len(raw) > LIMIT:
            raise ValueError('ROS sample exceeds 32 MiB limit')
        return compressor.compress(raw) if route['codec'] == 'zstd' else raw

    def decode(body):
        if bridge:
            wire = deserialize_message(body, CompressedImage)
            mat = cv2.imdecode(np.frombuffer(wire.data, np.uint8), cv2.IMREAD_COLOR)
            if mat is None:
                raise ValueError('Invalid JPEG')
            message = bridge.cv2_to_imgmsg(mat, encoding='bgr8')
            message.header = wire.header
            return message
        if route['codec'] == 'zstd' and zstd.frame_content_size(body) > LIMIT:
            raise ValueError('Zstd frame exceeds decoded size limit')
        raw = decompressor.decompress(body, max_output_size=LIMIT) if route['codec'] == 'zstd' else body
        if len(raw) > LIMIT:
            raise ValueError('Decoded sample too large')
        return deserialize_message(raw, kind)

    def socket():
        sock = context.socket(zmq.REP if sending else zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 1)
        sock.setsockopt(zmq.RCVHWM, 1)
        sock.setsockopt(zmq.MAXMSGSIZE, LIMIT)
        sock.setsockopt(zmq.SNDTIMEO, 200)
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.setsockopt(zmq.HEARTBEAT_IVL, 1000)
        sock.setsockopt(zmq.HEARTBEAT_TIMEOUT, 3000)
        if sending:
            sock.setsockopt_string(zmq.TCP_ACCEPT_FILTER, args.peer)
            sock.bind(f'tcp://{args.bind}:{route["port"]}')
        else:
            sock.setsockopt(zmq.IMMEDIATE, 1)
            sock.connect(f'tcp://{args.peer}:{route["port"]}')
        return sock

    sock = socket()
    epoch = uuid.uuid4().hex
    seen = None
    count = errors = stale = 0
    report = time.monotonic()
    next_poll = report
    node.get_logger().info(f'{"TX" if sending else "RX"} {route}; domain={os.getenv("ROS_DOMAIN_ID", "0")}')
    try:
        while rclpy.ok():
            try:
                if sending:
                    if not sock.poll(200):
                        continue
                    request = sock.recv_json()
                    header, body = {'version': 1, 'status': 'empty'}, b''
                    try:
                        if request != {'version': 1, 'stream': args.stream}:
                            raise ValueError('Wrong stream/protocol')
                        with lock:
                            message, arrival, sequence = latest
                        if message is not None and time.monotonic() - arrival < route['max_age']:
                            body = encode(message)
                            residence = time.monotonic() - arrival
                            if residence < route['max_age'] and len(body) <= LIMIT:
                                header.update(status='ok', epoch=epoch, sequence=sequence,
                                              residence=residence, codec=route['codec'], type=route['type'])
                            else:
                                body = b''
                                stale += 1
                    except Exception as exc:
                        header = {'version': 1, 'status': 'error', 'error': str(exc)}
                        body = b''
                        errors += 1
                    sock.send_multipart([json.dumps(header).encode(), body])
                    count += header['status'] == 'ok'
                else:
                    time.sleep(max(0, next_poll - time.monotonic()))
                    started = time.monotonic()
                    next_poll = started + 1 / route['rate']
                    sock.send_json({'version': 1, 'stream': args.stream})
                    if not sock.poll(max(200, int(route['max_age'] * 1000))):
                        raise TimeoutError('peer timeout')
                    parts = sock.recv_multipart()
                    if len(parts) != 2:
                        raise ValueError('Invalid reply framing')
                    header = json.loads(parts[0])
                    if header.get('version') != 1:
                        raise ValueError('Protocol version mismatch')
                    if header['status'] == 'error':
                        raise ValueError(header['error'])
                    if header['status'] == 'ok':
                        identity = (header['epoch'], header['sequence'])
                        if header['codec'] != route['codec'] or header['type'] != route['type']:
                            raise ValueError('Route configuration mismatch')
                        if identity != seen:
                            message = decode(parts[1])
                            # Conservative bound: source residence + entire request RTT/decode.
                            # No synchronized wall clocks needed for gateway transit age.
                            if header['residence'] + time.monotonic() - started <= route['max_age']:
                                pub.publish(message)
                                count += 1
                            else:
                                stale += 1
                            seen = identity
            except (zmq.ZMQError, TimeoutError) as exc:
                errors += 1
                sock.close()
                time.sleep(0.2)
                sock = socket()
            except Exception as exc:
                errors += 1
                node.get_logger().error(str(exc))
            now = time.monotonic()
            if now - report >= 5:
                node.get_logger().info(f'{count/(now-report):.1f} samples/s; errors/timeouts={errors}; stale={stale}')
                count = errors = stale = 0
                report = now
    finally:
        sock.close()
        context.term()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2)
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=['robot', 'pc'])
    parser.add_argument('--bind', required=True, help='Local interface IP; no wildcard bind')
    parser.add_argument('--peer', required=True, help='Other machine IP')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('routes.json'))
    parser.add_argument('--stream', help=argparse.SUPPRESS)
    args = parser.parse_args()
    import ipaddress
    ipaddress.IPv4Address(args.bind)
    ipaddress.IPv4Address(args.peer)
    if args.bind in ('*', '0.0.0.0'):
        parser.error('Bind to a specific local interface address')
    if os.getenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp') != 'rmw_cyclonedds_cpp':
        parser.error('Use RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    routes = json.loads(args.config.read_text())
    validate(routes)
    if args.stream:
        worker(args, routes[args.stream])
        return
    children = []
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        for name in routes:
            children.append(subprocess.Popen([sys.executable, __file__, *sys.argv[1:], '--stream', name]))
        while True:
            if any(child.poll() is not None for child in children):
                raise RuntimeError('Gateway worker exited; inspect its error above')
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
