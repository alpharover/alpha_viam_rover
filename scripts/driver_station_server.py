#!/usr/bin/env python3
"""Web-based driver station server (ROS 2 Humble).

Responsibilities:
- Serve a single-page web UI (static assets).
- Host a WebSocket endpoint for joystick control + rover stats.
- Publish Twist (or TwistStamped) to the configured cmd_vel topic with deadman safety.
- Subscribe to basic rover telemetry topics (power + wifi) and forward to the UI.

This is intentionally a simple, dependency-light implementation:
- Video remains MJPEG via `ustreamer` (very low latency on LAN).
- WebRTC is a future upgrade path (likely requires HTTPS for broad Safari compatibility).
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import os
import pathlib
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


try:
    from aiohttp import ClientSession, ClientTimeout, WSMsgType, http_exceptions, web
except Exception as exc:  # pragma: no cover
    print(
        "ERROR: aiohttp is not installed (required for the web driver station).\n"
        "Install on the rover with:\n"
        "  sudo apt-get update -y && sudo apt-get install -y python3-aiohttp\n",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


try:
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32, Int32, String
except Exception as exc:  # pragma: no cover
    print(
        "ERROR: ROS 2 Python deps are missing (rclpy, messages).\n"
        "Run this after sourcing ROS 2 Humble (and your overlay).\n",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


from lib.driver_station_mapping import scale_norm_twist


@dataclass(frozen=True)
class DriverStationConfig:
    # Web server
    http_bind: str
    http_port: int

    # Control publishing
    topic: str
    stamped: bool
    publish_rate_hz: float
    deadman_s: float
    frame_id: str

    # Stiction-aware command scaling
    min_speed: float
    min_turn: float
    default_max_speed: float
    default_max_turn: float
    default_deadband: float
    default_expo: float

    # Video (MJPEG)
    video_mode: str
    camera_device: str
    mjpeg_bind: str
    mjpeg_port: int
    mjpeg_source_size: str
    mjpeg_fps: float
    mjpeg_quality: int


@dataclass
class SharedState:
    lock: threading.Lock
    next_client_id: int

    # Control ownership and input
    controller_client_id: Optional[int]
    armed: bool
    lin_norm: float
    ang_norm: float
    last_cmd_monotonic: float

    # Tunables (may be updated live)
    max_speed: float
    max_turn: float
    deadband: float
    expo: float

    # Last published command (for UI)
    last_pub_linear_x: float
    last_pub_angular_z: float
    last_pub_monotonic: float

    # Telemetry topics
    power_bus_voltage_v: Optional[float]
    power_current_a: Optional[float]
    power_w: Optional[float]

    wifi_signal_dbm: Optional[float]
    wifi_link_ok: Optional[int]
    wifi_flap_count: Optional[int]
    wifi_iface: Optional[str]

    wheel_left_pos_rad: Optional[float]
    wheel_right_pos_rad: Optional[float]
    wheel_left_vel_rad_s: Optional[float]
    wheel_right_vel_rad_s: Optional[float]

    odom_linear_x_mps: Optional[float]
    odom_angular_z_rad_s: Optional[float]


def _configure_aiohttp_logging() -> None:
    class _DropBadStatusLine(logging.Filter):
        def filter(self, record: logging.LogRecord) -> bool:
            exc = record.exc_info[1] if record.exc_info else None
            if exc is not None and isinstance(exc, http_exceptions.BadStatusLine):
                return False
            return True

    flt = _DropBadStatusLine()
    logging.getLogger("aiohttp.server").addFilter(flt)
    logging.getLogger("aiohttp.web_protocol").addFilter(flt)


def _topic_type_matches(types: list[str], *, want: str) -> bool:
    return any(t == want for t in types)


def _detect_topic_stamped(node: Node, topic: str) -> Optional[bool]:
    """Return True/False when topic exists and is TwistStamped/Twist, else None."""

    by_name = {name: types for name, types in node.get_topic_names_and_types()}
    types = by_name.get(topic)
    if not types:
        return None
    if _topic_type_matches(types, want="geometry_msgs/msg/TwistStamped"):
        return True
    if _topic_type_matches(types, want="geometry_msgs/msg/Twist"):
        return False
    return None


def publish_twist(
    *,
    node: Node,
    pub,
    stamped: bool,
    linear_x: float,
    angular_z: float,
    frame_id: str,
) -> None:
    if stamped:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
    else:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
    pub.publish(msg)


class MjpegStreamer:
    def __init__(self, *, cfg: DriverStationConfig) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self._process: subprocess.Popen[bytes] | None = None
        self._error: str | None = None

    def get_status(self) -> tuple[bool, Optional[str]]:
        with self._lock:
            proc = self._process
            err = self._error

        if proc is None:
            return False, err

        alive = proc.poll() is None
        if alive:
            return True, None
        return False, err or "mjpeg stream stopped"

    @staticmethod
    def _quality_to_ustreamer(value: int) -> int:
        # Accept ffmpeg-style 2..31 and map to ustreamer 1..100.
        if 2 <= value <= 31:
            return int(round(100 - (value - 2) * 99 / 29))
        return max(1, min(100, value))

    def start(self) -> None:
        dev = self._cfg.camera_device
        if not os.path.exists(dev):
            with self._lock:
                self._error = f"camera device not found: {dev}"
            return

        ustreamer = shutil.which("ustreamer")
        if not ustreamer:
            with self._lock:
                self._error = "ustreamer not found (install: sudo apt-get install ustreamer)"
            return

        bind = self._cfg.mjpeg_bind
        port = int(self._cfg.mjpeg_port)
        size = str(self._cfg.mjpeg_source_size)
        fps = max(1.0, float(self._cfg.mjpeg_fps))
        quality = self._quality_to_ustreamer(int(self._cfg.mjpeg_quality))

        cmd = [
            ustreamer,
            "--host",
            bind,
            "--port",
            str(port),
            "--device",
            dev,
            "--resolution",
            size,
            "--format",
            "MJPEG",
            "--encoder",
            "HW",
            "--desired-fps",
            str(int(round(fps))),
            "--quality",
            str(quality),
            "--buffers=3",
            "--tcp-nodelay",
            "--exit-on-parent-death",
        ]

        try:
            with self._lock:
                self._process = subprocess.Popen(
                    cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
        except Exception as exc:
            with self._lock:
                self._error = f"failed to start ustreamer: {exc}"
            return

        time.sleep(0.25)
        with self._lock:
            proc = self._process
            if proc is not None and proc.poll() is not None:
                self._error = "ustreamer exited immediately (camera busy or unsupported format?)"

    def stop(self) -> None:
        with self._lock:
            proc = self._process
            self._process = None

        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


class DriverStationNode(Node):
    def __init__(self, *, cfg: DriverStationConfig, state: SharedState) -> None:
        super().__init__("alpha_viam_driver_station")
        self._cfg = cfg
        self._state = state

        # If topic already exists, auto-detect Twist vs TwistStamped unless user forced.
        stamped = cfg.stamped
        detected = _detect_topic_stamped(self, cfg.topic)
        if detected is not None:
            stamped = bool(detected)

        self._stamped = stamped
        self._pub = self.create_publisher(TwistStamped if stamped else Twist, cfg.topic, 10)

        self._sub_bus = self.create_subscription(Float32, "/power/bus_voltage", self._on_bus, 10)
        self._sub_cur = self.create_subscription(Float32, "/power/current", self._on_cur, 10)
        self._sub_pow = self.create_subscription(Float32, "/power/power", self._on_pow, 10)

        self._sub_rssi = self.create_subscription(Float32, "/wifi/signal_dBm", self._on_rssi, 10)
        self._sub_link = self.create_subscription(Int32, "/wifi/link_ok", self._on_link, 10)
        self._sub_flaps = self.create_subscription(Int32, "/wifi/flap_count", self._on_flaps, 10)
        self._sub_iface = self.create_subscription(String, "/wifi/iface", self._on_iface, 10)

        self._sub_joint_states = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 50
        )
        self._sub_odom_cm = self.create_subscription(
            Odometry, "/controller_manager/odom", self._on_odom, 50
        )
        self._sub_odom_dd = self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._on_odom, 50
        )

        period = 1.0 / max(1.0, float(cfg.publish_rate_hz))
        self._timer = self.create_timer(period, self._tick)

        msg_type = "TwistStamped" if self._stamped else "Twist"
        self.get_logger().info(
            f"driver_station publishing {msg_type} to {cfg.topic} at {cfg.publish_rate_hz:.0f} Hz (deadman {cfg.deadman_s:.2f}s)"
        )

    def _tick(self) -> None:
        now = time.monotonic()
        with self._state.lock:
            active = self._state.armed and (self._state.controller_client_id is not None)
            age = now - self._state.last_cmd_monotonic if self._state.last_cmd_monotonic else 999.0

            lin_norm = float(self._state.lin_norm)
            ang_norm = float(self._state.ang_norm)
            max_speed = float(self._state.max_speed)
            max_turn = float(self._state.max_turn)
            deadband = float(self._state.deadband)
            expo = float(self._state.expo)

        if not active or (self._cfg.deadman_s > 0.0 and age > self._cfg.deadman_s):
            lin_norm = 0.0
            ang_norm = 0.0

        linear_x, angular_z = scale_norm_twist(
            lin_norm=lin_norm,
            ang_norm=ang_norm,
            min_speed=self._cfg.min_speed,
            min_turn=self._cfg.min_turn,
            max_speed=max_speed,
            max_turn=max_turn,
            deadband=deadband,
            expo=expo,
        )

        publish_twist(
            node=self,
            pub=self._pub,
            stamped=self._stamped,
            linear_x=linear_x,
            angular_z=angular_z,
            frame_id=self._cfg.frame_id,
        )

        with self._state.lock:
            self._state.last_pub_linear_x = float(linear_x)
            self._state.last_pub_angular_z = float(angular_z)
            self._state.last_pub_monotonic = now

    def _on_bus(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.power_bus_voltage_v = float(msg.data)

    def _on_cur(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.power_current_a = float(msg.data)

    def _on_pow(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.power_w = float(msg.data)

    def _on_rssi(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.wifi_signal_dbm = float(msg.data)

    def _on_link(self, msg: Int32) -> None:
        with self._state.lock:
            self._state.wifi_link_ok = int(msg.data)

    def _on_flaps(self, msg: Int32) -> None:
        with self._state.lock:
            self._state.wifi_flap_count = int(msg.data)

    def _on_iface(self, msg: String) -> None:
        with self._state.lock:
            self._state.wifi_iface = str(msg.data)

    def _on_joint_states(self, msg: JointState) -> None:
        try:
            li = msg.name.index("left_wheel_joint")
            ri = msg.name.index("right_wheel_joint")
        except ValueError:
            return

        left_pos = float(msg.position[li]) if li < len(msg.position) else None
        right_pos = float(msg.position[ri]) if ri < len(msg.position) else None
        left_vel = float(msg.velocity[li]) if li < len(msg.velocity) else None
        right_vel = float(msg.velocity[ri]) if ri < len(msg.velocity) else None

        with self._state.lock:
            self._state.wheel_left_pos_rad = left_pos
            self._state.wheel_right_pos_rad = right_pos
            self._state.wheel_left_vel_rad_s = left_vel
            self._state.wheel_right_vel_rad_s = right_vel

    def _on_odom(self, msg: Odometry) -> None:
        with self._state.lock:
            self._state.odom_linear_x_mps = float(msg.twist.twist.linear.x)
            self._state.odom_angular_z_rad_s = float(msg.twist.twist.angular.z)


def _read_cpu_temp_c() -> Optional[float]:
    # Ubuntu on Pi generally exposes this.
    zone = pathlib.Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        if zone.exists():
            raw = zone.read_text(encoding="utf-8").strip()
            return float(raw) / 1000.0
    except Exception:
        pass
    return None


def _read_mem_used_mb() -> Optional[float]:
    meminfo = pathlib.Path("/proc/meminfo")
    try:
        if not meminfo.exists():
            return None
        total_kb = None
        avail_kb = None
        for line in meminfo.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("MemTotal:"):
                total_kb = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                avail_kb = int(line.split()[1])
        if total_kb is None or avail_kb is None:
            return None
        used_kb = max(0, total_kb - avail_kb)
        return used_kb / 1024.0
    except Exception:
        return None


def _system_stats() -> dict[str, Any]:
    load1 = None
    try:
        load1 = float(os.getloadavg()[0])
    except Exception:
        load1 = None

    return {
        "cpu_temp_c": _read_cpu_temp_c(),
        "load1": load1,
        "mem_used_mb": _read_mem_used_mb(),
    }


def _json_dumps(obj: Any) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False)


def _default_rover_hostname() -> str:
    host = os.environ.get("ROVER_HOSTNAME", "").strip()
    if host:
        return host
    h = socket.gethostname().strip() or "localhost"
    if "." not in h:
        h = h + ".local"
    return h


def _make_config_payload(
    *, cfg: DriverStationConfig, state: SharedState, request_host: str
) -> dict[str, Any]:
    host = request_host.split(":", 1)[0]
    mjpeg_stream_url = f"http://{host}:{cfg.mjpeg_port}/stream"
    mjpeg_root_url = f"http://{host}:{cfg.mjpeg_port}/"

    with state.lock:
        payload = {
            "type": "hello",
            "version": 1,
            "config": {
                "cmd_topic": cfg.topic,
                "stamped": bool(cfg.stamped),
                "publish_rate_hz": cfg.publish_rate_hz,
                "deadman_s": cfg.deadman_s,
                "min_speed": cfg.min_speed,
                "min_turn": cfg.min_turn,
                "max_speed": state.max_speed,
                "max_turn": state.max_turn,
                "deadband": state.deadband,
                "expo": state.expo,
                "video": {
                    "mode": cfg.video_mode,
                    "mjpeg_root_url": mjpeg_root_url,
                    "mjpeg_stream_url": mjpeg_stream_url,
                    "mjpeg_proxy_url": "/mjpeg",
                },
            },
        }
    return payload


async def ws_handler(request: web.Request) -> web.StreamResponse:
    app = request.app
    state: SharedState = app["state"]
    cfg: DriverStationConfig = app["cfg"]

    ws = web.WebSocketResponse(heartbeat=10.0)
    await ws.prepare(request)

    clients: set[web.WebSocketResponse] = app["clients"]

    with state.lock:
        client_id = int(state.next_client_id)
        state.next_client_id = client_id + 1

    clients.add(ws)
    try:
        hello = _make_config_payload(cfg=app["cfg"], state=state, request_host=request.host)
        await ws.send_str(_json_dumps(hello))

        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue

            try:
                data = json.loads(msg.data)
            except Exception:
                await ws.send_str(_json_dumps({"type": "error", "message": "invalid json"}))
                continue

            msg_type = str(data.get("type", ""))

            if msg_type == "arm":
                armed = bool(data.get("armed", False))
                with state.lock:
                    if armed:
                        if (
                            state.controller_client_id is None
                            or state.controller_client_id == client_id
                        ):
                            state.controller_client_id = client_id
                            state.armed = True
                            state.last_cmd_monotonic = time.monotonic()
                        else:
                            # Someone else holds control.
                            pass
                    else:
                        if state.controller_client_id == client_id:
                            state.controller_client_id = None
                        state.armed = False
                        state.lin_norm = 0.0
                        state.ang_norm = 0.0
                        state.last_cmd_monotonic = time.monotonic()
                continue

            if msg_type == "set_limits":
                max_speed = data.get("max_speed")
                max_turn = data.get("max_turn")
                deadband = data.get("deadband")
                expo = data.get("expo")
                with state.lock:
                    if max_speed is not None:
                        state.max_speed = max(float(cfg.min_speed), float(max_speed))
                    if max_turn is not None:
                        state.max_turn = max(float(cfg.min_turn), float(max_turn))
                    if deadband is not None:
                        state.deadband = float(max(0.0, min(0.99, float(deadband))))
                    if expo is not None:
                        state.expo = float(max(1e-6, float(expo)))
                continue

            if msg_type == "cmd":
                lin = data.get("lin", 0.0)
                ang = data.get("ang", 0.0)
                with state.lock:
                    if state.controller_client_id != client_id or not state.armed:
                        continue
                    state.lin_norm = float(max(-1.0, min(1.0, float(lin))))
                    state.ang_norm = float(max(-1.0, min(1.0, float(ang))))
                    state.last_cmd_monotonic = time.monotonic()
                continue

    finally:
        clients.discard(ws)
        with state.lock:
            if state.controller_client_id == client_id:
                state.controller_client_id = None
                state.armed = False
                state.lin_norm = 0.0
                state.ang_norm = 0.0
                state.last_cmd_monotonic = time.monotonic()

    return ws


async def stats_broadcast_loop(app: web.Application) -> None:
    state: SharedState = app["state"]
    cfg: DriverStationConfig = app["cfg"]
    clients: set[web.WebSocketResponse] = app["clients"]
    mjpeg: MjpegStreamer | None = app.get("mjpeg")

    hz = 5.0
    period = 1.0 / hz

    while True:
        await asyncio.sleep(period)

        running, mjpeg_err = (False, None)
        if mjpeg is not None:
            running, mjpeg_err = mjpeg.get_status()

        now_ms = int(time.time() * 1000)
        with state.lock:
            stats = {
                "power": {
                    "bus_voltage": state.power_bus_voltage_v,
                    "current": state.power_current_a,
                    "power": state.power_w,
                },
                "wifi": {
                    "signal_dbm": state.wifi_signal_dbm,
                    "link_ok": state.wifi_link_ok,
                    "flap_count": state.wifi_flap_count,
                    "iface": state.wifi_iface,
                },
                "drive": {
                    "wheel_left_pos_rad": state.wheel_left_pos_rad,
                    "wheel_right_pos_rad": state.wheel_right_pos_rad,
                    "wheel_left_vel_rad_s": state.wheel_left_vel_rad_s,
                    "wheel_right_vel_rad_s": state.wheel_right_vel_rad_s,
                    "wheel_vel_diff_rad_s": (
                        (state.wheel_left_vel_rad_s - state.wheel_right_vel_rad_s)
                        if state.wheel_left_vel_rad_s is not None
                        and state.wheel_right_vel_rad_s is not None
                        else None
                    ),
                },
                "odom": {
                    "linear_x": state.odom_linear_x_mps,
                    "angular_z": state.odom_angular_z_rad_s,
                },
                "system": _system_stats(),
                "control": {
                    "armed": bool(state.armed),
                    "controller_client_id": state.controller_client_id,
                    "cmd_linear": state.last_pub_linear_x,
                    "cmd_angular": state.last_pub_angular_z,
                    "deadman_s": cfg.deadman_s,
                    "last_cmd_age_s": (
                        (time.monotonic() - state.last_cmd_monotonic)
                        if state.last_cmd_monotonic
                        else None
                    ),
                },
                "video": {
                    "mode": cfg.video_mode,
                    "mjpeg_running": running,
                    "mjpeg_error": mjpeg_err,
                },
            }

        payload = _json_dumps({"type": "stats", "t": now_ms, "stats": stats})
        stale: list[web.WebSocketResponse] = []
        for ws in list(clients):
            try:
                await ws.send_str(payload)
            except Exception:
                stale.append(ws)
        for ws in stale:
            clients.discard(ws)


async def index_handler(request: web.Request) -> web.StreamResponse:
    web_dir: pathlib.Path = request.app["web_dir"]
    return web.FileResponse(path=str(web_dir / "index.html"))


async def config_handler(request: web.Request) -> web.StreamResponse:
    cfg: DriverStationConfig = request.app["cfg"]
    state: SharedState = request.app["state"]
    payload = _make_config_payload(cfg=cfg, state=state, request_host=request.host)
    return web.json_response(payload)


async def http_client_ctx(app: web.Application):
    timeout = ClientTimeout(total=None, sock_connect=2.0, sock_read=None)
    app["http_client"] = ClientSession(timeout=timeout)
    yield
    session: ClientSession | None = app.get("http_client")
    if session is not None:
        await session.close()


async def mjpeg_proxy_handler(request: web.Request) -> web.StreamResponse:
    cfg: DriverStationConfig = request.app["cfg"]
    mjpeg: MjpegStreamer | None = request.app.get("mjpeg")
    if cfg.video_mode != "mjpeg" or mjpeg is None:
        raise web.HTTPServiceUnavailable(text="mjpeg not available")

    session: ClientSession = request.app["http_client"]
    upstream = f"http://127.0.0.1:{cfg.mjpeg_port}/stream"

    try:
        async with session.get(upstream) as resp:
            if resp.status != 200:
                raise web.HTTPBadGateway(text="mjpeg upstream error")

            headers = {
                "Content-Type": resp.headers.get("Content-Type", "multipart/x-mixed-replace"),
                "Cache-Control": "no-store",
            }
            stream = web.StreamResponse(status=200, headers=headers)
            await stream.prepare(request)
            try:
                async for chunk in resp.content.iter_chunked(8192):
                    await stream.write(chunk)
            except ConnectionResetError:
                pass
            finally:
                with contextlib.suppress(Exception):
                    await stream.write_eof()
            return stream
    except asyncio.CancelledError:
        raise
    except Exception:
        raise web.HTTPBadGateway(text="mjpeg proxy error")


def _create_state(cfg: DriverStationConfig) -> SharedState:
    return SharedState(
        lock=threading.Lock(),
        next_client_id=1,
        controller_client_id=None,
        armed=False,
        lin_norm=0.0,
        ang_norm=0.0,
        last_cmd_monotonic=0.0,
        max_speed=max(float(cfg.default_max_speed), float(cfg.min_speed)),
        max_turn=max(float(cfg.default_max_turn), float(cfg.min_turn)),
        deadband=float(cfg.default_deadband),
        expo=float(cfg.default_expo),
        last_pub_linear_x=0.0,
        last_pub_angular_z=0.0,
        last_pub_monotonic=0.0,
        power_bus_voltage_v=None,
        power_current_a=None,
        power_w=None,
        wifi_signal_dbm=None,
        wifi_link_ok=None,
        wifi_flap_count=None,
        wifi_iface=None,
        wheel_left_pos_rad=None,
        wheel_right_pos_rad=None,
        wheel_left_vel_rad_s=None,
        wheel_right_vel_rad_s=None,
        odom_linear_x_mps=None,
        odom_angular_z_rad_s=None,
    )


def _parse_args(argv: list[str]) -> DriverStationConfig:
    parser = argparse.ArgumentParser(description="alpha_viam web driver station server")

    parser.add_argument("--http-bind", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8090)

    parser.add_argument("--topic", default="/controller_manager/cmd_vel_unstamped")
    parser.add_argument("--stamped", action="store_true")
    parser.add_argument("--unstamped", action="store_true")
    parser.add_argument("--rate", type=float, default=20.0, help="Twist publish rate (Hz)")
    parser.add_argument("--deadman", type=float, default=0.35, help="Deadman timeout (s)")
    parser.add_argument("--frame-id", default="base_link")

    parser.add_argument("--min-speed", type=float, default=0.72)
    parser.add_argument("--min-turn", type=float, default=0.0)
    parser.add_argument("--max-speed", type=float, default=1.0)
    parser.add_argument("--max-turn", type=float, default=6.5)
    parser.add_argument("--deadband", type=float, default=0.08)
    parser.add_argument("--expo", type=float, default=1.0)

    parser.add_argument("--video-mode", choices=("mjpeg", "off"), default="mjpeg")
    parser.add_argument("--camera-device", default="/dev/video0")
    parser.add_argument("--mjpeg-bind", default="0.0.0.0")
    parser.add_argument("--mjpeg-port", type=int, default=8080)
    parser.add_argument("--mjpeg-source-size", default="640x480")
    parser.add_argument("--mjpeg-fps", type=float, default=30.0)
    parser.add_argument("--mjpeg-quality", type=int, default=80)

    args = parser.parse_args(argv)

    if args.stamped and args.unstamped:
        raise SystemExit("ERROR: choose at most one of --stamped/--unstamped")

    if args.stamped:
        stamped = True
    elif args.unstamped:
        stamped = False
    else:
        stamped = False

    return DriverStationConfig(
        http_bind=str(args.http_bind),
        http_port=int(args.http_port),
        topic=str(args.topic),
        stamped=stamped,
        publish_rate_hz=max(1.0, float(args.rate)),
        deadman_s=max(0.0, float(args.deadman)),
        frame_id=str(args.frame_id),
        min_speed=max(0.0, float(args.min_speed)),
        min_turn=max(0.0, float(args.min_turn)),
        default_max_speed=max(0.0, float(args.max_speed)),
        default_max_turn=max(0.0, float(args.max_turn)),
        default_deadband=float(max(0.0, min(0.99, float(args.deadband)))),
        default_expo=float(max(1e-6, float(args.expo))),
        video_mode=str(args.video_mode),
        camera_device=str(args.camera_device),
        mjpeg_bind=str(args.mjpeg_bind),
        mjpeg_port=int(args.mjpeg_port),
        mjpeg_source_size=str(args.mjpeg_source_size),
        mjpeg_fps=max(1.0, float(args.mjpeg_fps)),
        mjpeg_quality=int(args.mjpeg_quality),
    )


async def run_server(
    cfg: DriverStationConfig, *, state: SharedState, mjpeg: MjpegStreamer | None
) -> int:
    web_dir = ROOT / "web" / "driver_station"
    if not web_dir.exists():
        print(f"ERROR: web assets not found at {web_dir}", file=sys.stderr)
        return 2

    app = web.Application()
    app["cfg"] = cfg
    app["state"] = state
    app["clients"] = set()
    app["web_dir"] = web_dir
    if mjpeg is not None:
        app["mjpeg"] = mjpeg

    app.cleanup_ctx.append(http_client_ctx)

    app.router.add_get("/", index_handler)
    app.router.add_get("/api/config", config_handler)
    app.router.add_get("/mjpeg", mjpeg_proxy_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static/", path=str(web_dir), show_index=False)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host=cfg.http_bind, port=cfg.http_port)

    try:
        await site.start()
    except OSError as exc:
        if getattr(exc, "errno", None) == 98:
            print(
                f"[driver_station] ERROR: port {cfg.http_port} already in use. "
                "Stop the existing driver station (`pkill -f driver_station_server.py`) "
                "or choose a different port with `--http-port`.",
                file=sys.stderr,
            )
            await runner.cleanup()
            return 2
        raise

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass

    stats_task = asyncio.create_task(stats_broadcast_loop(app))

    rover_host = _default_rover_hostname()
    url = f"http://{rover_host}:{cfg.http_port}/"
    print(f"[driver_station] UI: {url}")
    if cfg.video_mode == "mjpeg":
        print(f"[driver_station] Video (MJPEG UI): http://{rover_host}:{cfg.mjpeg_port}/")
        print(f"[driver_station] Video (MJPEG stream): http://{rover_host}:{cfg.mjpeg_port}/stream")

    try:
        await stop_event.wait()
    finally:
        stats_task.cancel()
        try:
            await stats_task
        except asyncio.CancelledError:
            pass
        await runner.cleanup()

    return 0


def main(argv: list[str]) -> int:
    cfg = _parse_args(argv)
    _configure_aiohttp_logging()
    state = _create_state(cfg)

    mjpeg: MjpegStreamer | None = None
    if cfg.video_mode == "mjpeg":
        mjpeg = MjpegStreamer(cfg=cfg)
        mjpeg.start()

    rclpy.init()
    node = DriverStationNode(cfg=cfg, state=state)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, name="driver_station_ros_spin", daemon=True)
    ros_thread.start()

    try:
        return asyncio.run(run_server(cfg, state=state, mjpeg=mjpeg))
    finally:
        if mjpeg is not None:
            mjpeg.stop()

        with state.lock:
            state.controller_client_id = None
            state.armed = False
            state.lin_norm = 0.0
            state.ang_norm = 0.0
            state.last_cmd_monotonic = time.monotonic()

        try:
            publish_twist(
                node=node,
                pub=node._pub,
                stamped=node._stamped,
                linear_x=0.0,
                angular_z=0.0,
                frame_id=cfg.frame_id,
            )
            time.sleep(0.05)
        except Exception:
            pass

        try:
            executor.shutdown()
        except Exception:
            pass

        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

        ros_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
