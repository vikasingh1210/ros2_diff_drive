#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String

import serial
import serial.tools.list_ports


def now_ms() -> int:
    return int(time.time() * 1000)


class SerialBridge(Node):
    """
    ROS2 <-> Arduino Bridge (PWM protocol)

    Subscriptions:
      /cmd_vel   (geometry_msgs/Twist)            -> computes per-wheel RPM, scales to PWM, sends "PWM:<L> <R>"
      /motor_cmd (std_msgs/Int32MultiArray [L,R]) -> sends "PWM:<L> <R>" directly

    Publications:
      /wheel_ticks (Int32MultiArray) [L,R]
      /wheel_rpm   (Float32MultiArray) [L,R]   (estimated from ticks at rpm_hz)
      /arduino_raw (String)                    (optional; off by default)

    Parameters:
      port:                 serial port (default /dev/ttyACM0). If empty, auto-detect Arduino-like ports.
      baud:                 default 115200
      wheel_radius:         meters (for /cmd_vel kinematics)
      wheel_base:           meters (distance between wheel centers)
      ticks_per_rev_left:   long (e.g., 620)
      ticks_per_rev_right:  long (e.g., 620)
      rpm_to_pwm_scale:     float multiplier to convert RPM (from /cmd_vel) -> PWM (e.g., 1.0 .. 3.0)
      idle_timeout_ms:      send PWM:0 0 if no cmd for this long (e.g., 500)
      read_hz:              serial read timer rate (e.g., 100 Hz)
      rpm_hz:               RPM computation rate (e.g., 20 Hz)
      publish_arduino_raw:  bool; if true, publishes /arduino_raw lines
    """

    def __init__(self):
        super().__init__("arduino_bridge")

        # --- Parameters ---
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", 0.03)
        self.declare_parameter("wheel_base", 0.16)
        self.declare_parameter("ticks_per_rev_left", 620)
        self.declare_parameter("ticks_per_rev_right", 620)
        self.declare_parameter("rpm_to_pwm_scale", 1.0)
        self.declare_parameter("idle_timeout_ms", 500)
        self.declare_parameter("read_hz", 100.0)
        self.declare_parameter("rpm_hz", 20.0)
        self.declare_parameter("publish_arduino_raw", False)

        self.port: str = self.get_parameter("port").get_parameter_value().string_value
        self.baud: int = int(self.get_parameter("baud").value)
        self.r: float = float(self.get_parameter("wheel_radius").value)
        self.wb: float = float(self.get_parameter("wheel_base").value)
        self.tprL: int = int(self.get_parameter("ticks_per_rev_left").value)
        self.tprR: int = int(self.get_parameter("ticks_per_rev_right").value)
        self.rpm_to_pwm: float = float(self.get_parameter("rpm_to_pwm_scale").value)
        self.idle_timeout_ms: int = int(self.get_parameter("idle_timeout_ms").value)
        self.read_hz: float = float(self.get_parameter("read_hz").value)
        self.rpm_hz: float = float(self.get_parameter("rpm_hz").value)
        self.publish_raw: bool = bool(self.get_parameter("publish_arduino_raw").value)

        # Auto-detect/open serial
        self.ser: Optional[serial.Serial] = None
        self._open_serial()

        # --- ROS I/O ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.sub_motor = self.create_subscription(Int32MultiArray, "/motor_cmd", self.motor_cb, 10)

        self.pub_ticks = self.create_publisher(Int32MultiArray, "/wheel_ticks", qos)
        self.pub_rpm = self.create_publisher(Float32MultiArray, "/wheel_rpm", qos)
        self.pub_raw = self.create_publisher(String, "/arduino_raw", qos) if self.publish_raw else None

        # RPM estimation state
        self.have_ticks = False
        self._last_ticks_L = 0
        self._last_ticks_R = 0
        self._prev_for_rpm = None

        # Watchdog
        self.last_cmd_time_ms = now_ms()
        self.last_sent_pwm = (0, 0)

        # Serial input buffer
        self.rx_buf = ""

        # Timers
        self.read_timer = self.create_timer(1.0 / self.read_hz, self._read_serial_timer)
        self.rpm_timer = self.create_timer(1.0 / self.rpm_hz, self._rpm_timer)
        self.watchdog_timer = self.create_timer(0.05, self._watchdog_timer)  # 20 Hz

        self.get_logger().info(
            f"Arduino bridge ready. Port={self.port or '(auto)'} @ {self.baud}, "
            f"r={self.r} m, wb={self.wb} m, TPR L/R={self.tprL}/{self.tprR}, "
            f"rpm_to_pwm={self.rpm_to_pwm}, idle_timeout={self.idle_timeout_ms} ms"
        )

    # ---------------- Serial helpers ----------------

    def _auto_detect_port(self) -> Optional[str]:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            name = (p.device or "").lower()
            desc = (p.description or "").lower()
            if "arduino" in desc or name.startswith("/dev/ttyacm") or name.startswith("/dev/ttyusb"):
                return p.device
        return None

    def _open_serial(self):
        port_to_try = self.port
        ser = None
        try:
            if not port_to_try:
                port_to_try = self._auto_detect_port()
            if port_to_try:
                ser = serial.Serial(port_to_try, self.baud, timeout=0.02)
                self.port = port_to_try
        except Exception as e:
            self.get_logger().warn(f"Serial open failed on {port_to_try}: {e}")

        if ser and ser.is_open:
            self.ser = ser
            # Give Arduino time to reset if needed
            time.sleep(2.0)
            self.get_logger().info(f"Connected serial {self.port} @ {self.baud}")
        else:
            self.ser = None
            self.get_logger().error("No serial connection. Will retry periodically.")

    def _ensure_serial(self):
        if self.ser and self.ser.is_open:
            return
        self._open_serial()

    # ---------------- Subscribers ----------------

    def cmd_cb(self, msg: Twist):
        """Convert /cmd_vel (m/s, rad/s) -> per-wheel RPM -> PWM (scaled) -> send."""
        v = float(msg.linear.x)
        wz = float(msg.angular.z)

        # diff-drive wheel angular velocities (rad/s)
        w_l = (v - (wz * self.wb / 2.0)) / self.r
        w_r = (v + (wz * self.wb / 2.0)) / self.r

        # rad/s -> RPM
        rpm_l = (w_l * 60.0) / (2.0 * math.pi)
        rpm_r = (w_r * 60.0) / (2.0 * math.pi)

        # scale RPM to PWM command
        pwm_l = int(rpm_l * self.rpm_to_pwm)
        pwm_r = int(rpm_r * self.rpm_to_pwm)

        self._send_lr("PWM", pwm_l, pwm_r)
        self.get_logger().info(f"/cmd_vel v={v:.3f} wz={wz:.3f} -> rpm L/R={int(rpm_l)}/{int(rpm_r)} -> pwm L/R={pwm_l}/{pwm_r}")

    def motor_cb(self, msg: Int32MultiArray):
        """Direct per-wheel PWM command: /motor_cmd [L, R]."""
        if not msg.data or len(msg.data) < 2:
            self.get_logger().warn("motor_cmd needs [left_pwm, right_pwm]")
            return
        L = int(msg.data[0])
        R = int(msg.data[1])
        self._send_lr("PWM", L, R)
        self.get_logger().info(f"/motor_cmd -> pwm L/R={L}/{R}")

    # ---------------- Send helpers ----------------

    def _send_lr(self, prefix: str, L: int, R: int):
        self._ensure_serial()
        if not self.ser:
            return
        # clamp to Arduino range
        L = max(-255, min(255, int(L)))
        R = max(-255, min(255, int(R)))
        line = f"{prefix}:{L} {R}\n"
        try:
            self.ser.write(line.encode("utf-8"))
            self.last_cmd_time_ms = now_ms()
            self.last_sent_pwm = (L, R)
            self.get_logger().info(f"-> {line.strip()}")
        except Exception as e:
            self.get_logger().warn(f"serial write failed: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _send_stop(self):
        if self.last_sent_pwm != (0, 0):
            self._send_lr("PWM", 0, 0)
            self.get_logger().info("Watchdog: sent PWM:0 0")

    # ---------------- Timers ----------------

    def _read_serial_timer(self):
        """Read lines from Arduino. Expect:
           TCK:<left_ticks> <right_ticks>
           (Optionally) RPM:<left_rpm> <right_rpm>
        """
        self._ensure_serial()
        if not self.ser:
            return
        try:
            n = self.ser.in_waiting
            if n > 0:
                chunk = self.ser.read(n).decode("utf-8", errors="ignore")
                self.rx_buf += chunk

            while "\n" in self.rx_buf:
                line, self.rx_buf = self.rx_buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                if self.pub_raw is not None:
                    self.pub_raw.publish(String(data=line))

                if line.startswith("TCK:"):
                    body = line[4:].strip()
                    parts = body.split()
                    if len(parts) >= 2:
                        try:
                            tl = int(parts[0])
                            tr = int(parts[1])
                            self._publish_ticks(tl, tr)
                        except ValueError:
                            self.get_logger().warn(f"Bad TCK line: {line}")

                elif line.startswith("RPM:"):
                    body = line[4:].strip()
                    parts = body.split()
                    if len(parts) >= 2:
                        try:
                            rl = float(parts[0])
                            rr = float(parts[1])
                            self._publish_rpm(rl, rr)
                        except ValueError:
                            self.get_logger().warn(f"Bad RPM line: {line}")

                else:
                    # Debug/ignore unknown lines
                    self.get_logger().debug(f"Arduino: {line}")

        except Exception as e:
            self.get_logger().warn(f"serial read failed: {e}")
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _rpm_timer(self):
        """Compute RPM from tick deltas at fixed rate; publish /wheel_rpm."""
        if not self.have_ticks:
            return
        dt = 1.0 / self.rpm_hz
        tl = self._last_ticks_L
        tr = self._last_ticks_R

        if self._prev_for_rpm is None:
            self._prev_for_rpm = (tl, tr)
            return

        pL, pR = self._prev_for_rpm
        dL = tl - pL
        dR = tr - pR
        self._prev_for_rpm = (tl, tr)

        try:
            rpmL = float(dL) * (60.0 / (self.tprL * dt))
            rpmR = float(dR) * (60.0 / (self.tprR * dt))
            self._publish_rpm(rpmL, rpmR)
        except ZeroDivisionError:
            pass

    def _watchdog_timer(self):
        """If no commands for idle_timeout_ms, send STOP."""
        if (now_ms() - self.last_cmd_time_ms) > self.idle_timeout_ms:
            self._send_stop()

    # ---------------- Publishers ----------------

    def _publish_ticks(self, tl: int, tr: int):
        msg = Int32MultiArray()
        msg.data = [int(tl), int(tr)]
        self.pub_ticks.publish(msg)

        self._last_ticks_L = int(tl)
        self._last_ticks_R = int(tr)
        self.have_ticks = True

    def _publish_rpm(self, rl: float, rr: float):
        msg = Float32MultiArray()
        msg.data = [float(rl), float(rr)]
        self.pub_rpm.publish(msg)


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
