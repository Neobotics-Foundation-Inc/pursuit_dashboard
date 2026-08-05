#!/usr/bin/env python3
"""Neoracer pure pursuit service: chase AR tags or cars, tuning dashboard.

Same pattern as the wallfollow dashboard (stdlib HTTP + rclpy) on port 8084.
Two stages, selected by `mode` in pursuit.yaml (file edit only, no UI toggle):
  0: follow AR tags. ArUco DICT_6X6_250, detected here from the camera,
     the same dictionary the neoracer library uses.
  1: follow cars. Reads the driver inference node's YOLO detections
     (vision_msgs/Detection2DArray on /edgetpu/inference).

Control is identical once a target box exists: steer at the box center with
a PD on horizontal offset, drive at `speed` and brake as the box area grows
toward `safety_area` (full stop at or beyond it). No target means stop.

CAUTION: the neoracer mux forwards /drive with no ROS deadman; the physical
gate is the SWB switch on the transmitter. The shipped yaml has speed 0.0.
Steering is negated on publish: wire positive turns this car left
(physically verified on the wallfollow service).
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import yaml

PORT = 8084
BASE = Path(__file__).resolve().parent
YAML_PATH = BASE / 'pursuit.yaml'

DETECT_RATE_HZ = 15.0   # cap for on-board ArUco detection
STALE_SEC = 0.5         # no target this long -> publish zeros

_lock = threading.Lock()
_params: dict = {}
_state: dict = {'detections': [], 'target': None, 'error': None,
                'steer': 0.0, 'speed_cmd': 0.0, 'res': [0, 0], 'seen': False}
_preview = b''


def load_params():
    with _lock:
        _params.update(yaml.safe_load(YAML_PATH.read_text()))
        _params['speed'] = max(0.0, min(1.0, float(_params['speed'])))
        _params['mode'] = 1 if str(_params['mode']).strip() == '1' else 0


def save_params():
    """Rewrite values in place so the yaml comments survive."""
    text = YAML_PATH.read_text()
    with _lock:
        for k, v in _params.items():
            if isinstance(v, (int, float)) and k != 'mode':
                text = re.sub(rf'(?m)^{k}:\s*[-\d.eE+]+', f'{k}: {v}', text)
    YAML_PATH.write_text(text)


def _make_aruco_detector():
    """Handle both the old and new cv2 aruco APIs."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda gray: det.detectMarkers(gray)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


class PursuitNode(Node):
    def __init__(self):
        super().__init__('pursuit')
        with _lock:
            cam_topic = _params['camera_topic']
            det_topic = _params['detections_topic']
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(Image, cam_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, det_topic, self._det_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self._aruco = _make_aruco_detector()
        self._last_detect = 0.0
        self._last_target = 0.0
        self._last_error = 0.0
        self._last_cmd = 0.0
        self._speed_cmd = 0.0
        self._res = (640, 480)

    # ---- inputs ----

    def _image_cb(self, msg):
        global _preview
        if msg.encoding == 'jpeg':
            arr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if arr is None:
            return
        h, w = arr.shape[:2]
        self._res = (w, h)
        with _lock:
            p = dict(_params)
        now = time.monotonic()
        if now - self._last_detect >= 1.0 / DETECT_RATE_HZ:
            self._last_detect = now
            pw = int(p['preview_width'])
            small = cv2.resize(arr, (pw, max(1, round(pw * h / w))))
            quality = [cv2.IMWRITE_JPEG_QUALITY, int(p['preview_quality'])]
            with _lock:
                _preview = cv2.imencode('.jpg', small, quality)[1].tobytes()
            if p['mode'] == 0:
                self._control(self._find_tags(arr), p)

    def _det_cb(self, msg):
        with _lock:
            p = dict(_params)
        if p['mode'] != 1:
            return
        dets = []
        for d in msg.detections:
            score, label = 1.0, ''
            if d.results:
                score = float(d.results[0].hypothesis.score)
                label = str(d.results[0].hypothesis.class_id)
            dets.append({'cx': d.bbox.center.position.x, 'cy': d.bbox.center.position.y,
                         'w': d.bbox.size_x, 'h': d.bbox.size_y,
                         'score': round(score, 3), 'label': label})
        self._control(dets, p)

    def _find_tags(self, arr):
        gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
        corners, ids = self._aruco(gray)
        dets = []
        for i, quad in enumerate(corners):
            xs, ys = quad[0][:, 0], quad[0][:, 1]
            dets.append({'cx': float(xs.mean()), 'cy': float(ys.mean()),
                         'w': float(xs.max() - xs.min()), 'h': float(ys.max() - ys.min()),
                         'score': 1.0,
                         'label': f'tag {int(ids[i][0])}' if ids is not None else 'tag'})
        return dets

    # ---- control ----

    def _control(self, dets, p):
        W, H = self._res
        dets = [d for d in dets if d['score'] >= p['score_threshold']]
        target = max(dets, key=lambda d: d['w'] * d['h'], default=None)

        if target is None:
            # Detection drops out for a frame or two all the time (motion
            # blur, tag angle). Slamming the throttle to zero on each miss
            # and restarting the ramp reads as jerking, so coast on the last
            # command through a short grace window and only stop when the
            # target is genuinely gone.
            if time.monotonic() - self._last_target > STALE_SEC:
                self._publish_stopped(dets)
            else:
                out = AckermannDriveStamped()
                out.drive.speed = float(self._speed_cmd)
                out.drive.steering_angle = float(-self._last_cmd)
                self._pub.publish(out)
                self._update_state(dets, None, None, self._last_cmd, None)
            return
        self._last_target = time.monotonic()

        error = (target['cx'] - W / 2) / (W / 2)  # -1 left .. +1 right
        cmd = p['angle_kp'] * error + p['angle_kd'] * (error - self._last_error)
        self._last_error = error
        cmd = max(-1.0, min(1.0, cmd))
        self._last_cmd = cmd

        # Brake as the box area approaches safety_area; stop at or past it.
        area = (target['w'] * target['h']) / (W * H)
        target_speed = p['speed'] * max(0.0, 1.0 - area / max(p['safety_area'], 1e-3))
        self._speed_cmd += p['speed_kp'] * (target_speed - self._speed_cmd)
        self._speed_cmd = max(0.0, min(1.0, self._speed_cmd))

        out = AckermannDriveStamped()
        out.drive.speed = float(self._speed_cmd)
        out.drive.steering_angle = float(-cmd)  # wire positive = left on this car
        self._pub.publish(out)
        self._update_state(dets, target, error, cmd, area)

    def _publish_stopped(self, dets):
        self._speed_cmd = 0.0
        self._pub.publish(AckermannDriveStamped())
        self._update_state(dets, None, None, 0.0, None)

    def _watchdog(self):
        """No target for STALE_SEC (camera died, tag gone): keep zeros flowing."""
        if time.monotonic() - self._last_target > STALE_SEC and self._speed_cmd > 0:
            self._publish_stopped([])

    def _update_state(self, dets, target, error, cmd, area):
        with _lock:
            _state.update({
                'detections': dets, 'target': target,
                'error': None if error is None else round(error, 3),
                'steer': round(cmd, 3), 'speed_cmd': round(self._speed_cmd, 3),
                'area': None if area is None else round(area, 4),
                'res': list(self._res), 'seen': target is not None,
            })


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self):
        if self.path == '/':
            self._send((BASE / 'pursuit.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path.startswith('/frame'):
            with _lock:
                data = _preview
            if data:
                self._send(data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path == '/state':
            with _lock:
                body = json.dumps({**_state, 'params': _params})
            self._send(body.encode(), 'application/json')
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/params':
            data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            with _lock:
                for k in ('speed', 'speed_kp', 'safety_area', 'angle_kp',
                          'angle_kd', 'score_threshold'):
                    if k in data:
                        _params[k] = float(data[k])
                _params['speed'] = max(0.0, min(1.0, _params['speed']))
        elif self.path == '/save':
            save_params()
        elif self.path == '/load':
            load_params()
        else:
            self.send_error(404)
            return
        self._send(b'ok', 'text/plain')

    def _send(self, body, ctype, cache=None):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def main():
    load_params()
    rclpy.init()
    node = PursuitNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    with _lock:
        mode = _params['mode']
    print(f'Pure pursuit dashboard on http://0.0.0.0:{PORT} (mode={mode})')
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()


if __name__ == '__main__':
    main()
