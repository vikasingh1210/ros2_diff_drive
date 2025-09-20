#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
)
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GyroDistance(Node):
    """
    IMU-only forward distance estimator:

    - Auto IMU topic discovery (sensor_msgs/Imu)
    - Dual QoS subscribers (SensorData + Reliable)
    - Accel scale (+ optional auto-cal at startup), Gyro scale
    - Initial roll/pitch from gravity at startup
    - Gravity compensation, LPF + deadband
    - ZUPT (+ bias learning)
    - Movement gating with hysteresis (won't integrate at rest)
    - Max speed clamp (safety)
    - distance_gain to calibrate overall scale
    - Optional signed distance (vs. default monotonic path length)
    """

    def __init__(self):
        super().__init__('gyro_distance')

        # ---------- Parameters ----------
        self.declare_parameter('imu_topic', '')               # blank => auto-discover
        self.declare_parameter('forward_axis', 'x')           # 'x' or 'y' (robot forward axis in IMU frame)
        self.declare_parameter('axis_sign', 1)                # +1 or -1 to flip forward direction

        self.declare_parameter('accel_lpf_hz', 8.0)           # accel LPF cutoff (Hz)
        self.declare_parameter('accel_deadband', 0.05)        # m/s^2 small-noise kill

        self.declare_parameter('use_zupt', True)
        self.declare_parameter('zupt_acc_eps', 0.03)          # m/s^2
        self.declare_parameter('zupt_gyro_eps', 0.2)          # rad/s (forgiving)
        self.declare_parameter('zupt_hold_ms', 200)           # ms still before snap v=0
        self.declare_parameter('bias_learn_alpha', 0.02)      # bias learning rate (0..1)

        # Units/scaling
        self.declare_parameter('accel_scale', 1.0)            # multiply raw accel (e.g., 9.80665 if driver publishes g)
        self.declare_parameter('auto_accel_scale', True)      # learn accel_scale at startup (~2s still)
        self.declare_parameter('auto_calib_duration_s', 2.0)

        # Many drivers already publish rad/s; if your gyro is deg/s, set 0.0174533
        self.declare_parameter('gyro_scale', 1.0)

        # Initial attitude
        self.declare_parameter('assume_still_on_start', True)
        self.declare_parameter('init_att_calib_s', 1.5)

        # Movement gating (hysteresis): integrate only when truly moving
        self.declare_parameter('integrate_only_when_moving', True)
        self.declare_parameter('start_accel_thresh', 0.10)    # m/s^2 to start integrating
        self.declare_parameter('stop_accel_thresh', 0.06)     # m/s^2 to stop integrating

        # Safety: cap velocity to plausible robot max speed (m/s)
        self.declare_parameter('max_speed_mps', 3.0)

        # Distance calibration
        self.declare_parameter('distance_gain', 1.0)          # multiply integrated distance (tape-test gain)
        self.declare_parameter('signed_distance', False)       # False = path length (monotonic), True = signed displacement

        # ---------- Read parameters ----------
        p = self.get_parameter
        self.cfg_topic   = str(p('imu_topic').value or '')
        self.axis        = str(p('forward_axis').value).lower()
        self.sign        = int(p('axis_sign').value)

        self.acc_lpf_hz   = float(p('accel_lpf_hz').value)
        self.acc_deadband = float(p('accel_deadband').value)

        self.use_zupt      = bool(p('use_zupt').value)
        self.zupt_acc_eps  = float(p('zupt_acc_eps').value)
        self.zupt_gyro_eps = float(p('zupt_gyro_eps').value)
        self.zupt_hold_ms  = int(p('zupt_hold_ms').value)
        self.bias_alpha    = float(p('bias_learn_alpha').value)

        self.accel_scale   = float(p('accel_scale').value)
        self.auto_accel_scale = bool(p('auto_accel_scale').value)
        self.auto_calib_duration_s = float(p('auto_calib_duration_s').value)

        self.gyro_scale    = float(p('gyro_scale').value)
        self.att_calib_enabled = bool(p('assume_still_on_start').value)
        self.init_att_calib_s  = float(p('init_att_calib_s').value)

        self.integrate_only_when_moving = bool(p('integrate_only_when_moving').value)
        self.start_accel_thresh = float(p('start_accel_thresh').value)
        self.stop_accel_thresh  = float(p('stop_accel_thresh').value)

        self.max_speed_mps = float(p('max_speed_mps').value)
        self.distance_gain = float(p('distance_gain').value)
        self.signed_distance = bool(p('signed_distance').value)

        if self.axis not in ('x', 'y'):
            self.axis = 'x'
        if self.sign not in (-1, 1):
            self.sign = 1

        # ---------- Publishers ----------
        self.pub_dist = self.create_publisher(Float32, '/distance_imu', 10)
        self.pub_vel  = self.create_publisher(Float32, '/vel_imu', 10)

        # ---------- Auto-discovery subscription (deferred) ----------
        self.sub_imu_best = None
        self.sub_imu_rel  = None
        self.selected_topic = None
        self.create_timer(0.5, self._ensure_subscription)

        # ---------- State ----------
        self.last_time = None

        # attitude (roll/pitch)
        self.roll = 0.0
        self.pitch = 0.0

        # kinematics
        self.v_fwd = 0.0
        self.dist = 0.0
        self.a_prev = 0.0
        self.bias = 0.0
        self.still_since = None

        # accel filter
        self.a_lpf = 0.0
        self.lpf_init = False

        # auto accel-scale
        self.calib_started = None
        self.calib_samples = 0
        self.calib_sum_norm = 0.0
        self.calib_done = (not self.auto_accel_scale)

        # initial attitude calibration
        self._att_calib_done = (not self.att_calib_enabled)
        self._att_calib_end  = None
        self._att_sum_ax = 0.0
        self._att_sum_ay = 0.0
        self._att_sum_az = 0.0
        self._att_n = 0

        # movement gate
        self.moving = False

        self.get_logger().info(
            "gyro_distance start: "
            f"topic='{self.cfg_topic or 'auto'}', axis={self.axis}{'+' if self.sign>0 else '-'}, "
            f"LPF={self.acc_lpf_hz}Hz, ZUPT acc<{self.zupt_acc_eps} gyro<{self.zupt_gyro_eps}, "
            f"accel_scale={self.accel_scale}, auto_scale={'on' if self.auto_accel_scale else 'off'}, "
            f"gyro_scale={self.gyro_scale}, init_att={'on' if self.att_calib_enabled else 'off'}({self.init_att_calib_s}s), "
            f"gate={'on' if self.integrate_only_when_moving else 'off'}, vmax={self.max_speed_mps} m/s, "
            f"distance_gain={self.distance_gain}, signed_distance={self.signed_distance}"
        )

    # ---------- Auto-discover IMU topic and subscribe with BOTH QoS ----------
    def _ensure_subscription(self):
        if self.sub_imu_best or self.sub_imu_rel:
            return

        candidates = []
        if self.cfg_topic:
            candidates.append(self.cfg_topic)
        candidates += ['/imu', '/imu/data_raw', '/mpu6050/imu', '/mpu6050/imu/data_raw']

        seen = set(candidates)
        for name, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/Imu' in types and name not in seen:
                candidates.append(name)
                seen.add(name)

        chosen = None
        for t in candidates:
            try:
                pubs = self.get_publishers_info_by_topic(t)
                if len(pubs) > 0:
                    chosen = t
                    break
            except Exception:
                continue

        if not chosen:
            self.get_logger().warn("No sensor_msgs/Imu publishers found yet; will retry...")
            return

        self.selected_topic = chosen
        # BEST_EFFORT (SensorData)
        self.sub_imu_best = self.create_subscription(Imu, chosen, self._imu_cb, qos_profile_sensor_data)
        # RELIABLE
        self.sub_imu_rel  = self.create_subscription(
            Imu, chosen, self._imu_cb,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        )
        pubs = self.get_publishers_info_by_topic(chosen)
        self.get_logger().info(f"Subscribed to IMU topic: {chosen} (publishers={len(pubs)})")

    # ---------- IMU callback ----------
    def _imu_cb(self, msg: Imu):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.calib_started = now
            if self.att_calib_enabled and self._att_calib_end is None:
                self._att_calib_end = now + Duration(seconds=self.init_att_calib_s)
                self.get_logger().info(
                    f"Calibrating initial attitude for {self.init_att_calib_s:.1f}s; keep robot still..."
                )
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.25:
            self.last_time = now
            return

        # raw accel & gyro
        ax_raw = float(msg.linear_acceleration.x)
        ay_raw = float(msg.linear_acceleration.y)
        az_raw = float(msg.linear_acceleration.z)
        gx = self.gyro_scale * float(msg.angular_velocity.x)
        gy = self.gyro_scale * float(msg.angular_velocity.y)
        gz = self.gyro_scale * float(msg.angular_velocity.z)

        # ---- auto accel-scale ----
        if not self.calib_done and self.calib_started is not None:
            self.calib_samples += 1
            norm = math.sqrt(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw)
            self.calib_sum_norm += norm
            if (now - self.calib_started) >= Duration(seconds=self.auto_calib_duration_s):
                avg = self.calib_sum_norm / max(1, self.calib_samples)
                if avg > 1e-3:
                    scale = 9.80665 / avg
                    self.accel_scale = clamp(scale, 0.5, 20.0)
                    self.get_logger().info(
                        f"Auto accel_scale calibrated: {self.accel_scale:.3f} (avg_norm={avg:.3f})"
                    )
                else:
                    self.get_logger().warn("Auto accel_scale failed; keeping current accel_scale.")
                self.calib_done = True

        # apply accel scale
        ax = self.accel_scale * ax_raw
        ay = self.accel_scale * ay_raw
        az = self.accel_scale * az_raw

        # ---- initial attitude calibration (roll/pitch from gravity) ----
        if not self._att_calib_done:
            self._att_sum_ax += ax
            self._att_sum_ay += ay
            self._att_sum_az += az
            self._att_n += 1

            if now >= self._att_calib_end and self._att_n > 0:
                axm = self._att_sum_ax / self._att_n
                aym = self._att_sum_ay / self._att_n
                azm = self._att_sum_az / self._att_n
                gmag = math.sqrt(axm*axm + aym*aym + azm*azm) + 1e-9
                axn, ayn, azn = axm / gmag, aym / gmag, azm / gmag
                pitch0 = math.asin(-clamp(axn, -1.0, 1.0))
                roll0  = math.atan2(ayn, azn)
                self.pitch = pitch0
                self.roll  = roll0
                self._att_calib_done = True
                self.v_fwd = 0.0
                self.a_prev = 0.0
                self.still_since = None
                self.get_logger().info(
                    f"Initial attitude set: roll={self.roll:.3f} rad, pitch={self.pitch:.3f} rad"
                )
            self.last_time = now
            return

        # ---- Complementary filter for roll/pitch ----
        self.roll  += gx * dt
        self.pitch += gy * dt
        g_mag = math.sqrt(ax*ax + ay*ay + az*az) + 1e-9
        axn, ayn, azn = ax / g_mag, ay / g_mag, az / g_mag
        pitch_meas = math.asin(-clamp(axn, -1.0, 1.0))
        roll_meas  = math.atan2(ayn, azn)
        k_corr = 0.02
        self.roll  = (1.0 - k_corr) * self.roll  + k_corr * roll_meas
        self.pitch = (1.0 - k_corr) * self.pitch + k_corr * pitch_meas

        # ---- Gravity compensation (exact body gravity from roll/pitch) ----
        g = 9.80665
        sp, cp = math.sin(self.pitch), math.cos(self.pitch)
        sr, cr = math.sin(self.roll),  math.cos(self.roll)
        g_bx = -g * sp
        g_by =  g * sr * cp
        ax_lin = ax - g_bx
        ay_lin = ay - g_by

        # ---- Choose forward axis ----
        a_fwd = self.sign * (ax_lin if self.axis == 'x' else ay_lin)

        # ---- LPF + deadband ----
        alpha = 2.0 * math.pi * max(0.1, self.acc_lpf_hz) * dt
        alpha = clamp(alpha, 0.0, 1.0)
        if not self.lpf_init:
            self.a_lpf = a_fwd
            self.lpf_init = True
        else:
            self.a_lpf += alpha * (a_fwd - self.a_lpf)
        a_clean = 0.0 if abs(self.a_lpf) < self.acc_deadband else self.a_lpf

        # ---- Movement gate (hysteresis) ----
        gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
        if self.integrate_only_when_moving:
            if not self.moving:
                if abs(a_clean) > self.start_accel_thresh:
                    self.moving = True
            else:
                # require both low accel and reasonably low gyro to stop
                if abs(a_clean) < self.stop_accel_thresh and gyro_norm < self.zupt_gyro_eps * 1.5:
                    self.moving = False

        # ---- ZUPT (accel + gyro) ----
        stopped = False
        if self.use_zupt:
            if abs(a_clean) < self.zupt_acc_eps and gyro_norm < self.zupt_gyro_eps:
                if self.still_since is None:
                    self.still_since = now
                elif (now - self.still_since) >= Duration(seconds=self.zupt_hold_ms / 1000.0):
                    stopped = True
            else:
                self.still_since = None

        if stopped:
            # learn accel bias, clamp velocity, and mark not moving
            self.bias = (1.0 - self.bias_alpha) * self.bias + self.bias_alpha * self.a_lpf
            self.v_fwd = 0.0
            self.moving = False

        # If not moving, zero velocity & publish — no integration.
        if self.integrate_only_when_moving and not self.moving:
            # keep learning bias slowly even when not "stopped"
            self.bias = (1.0 - self.bias_alpha) * self.bias + self.bias_alpha * self.a_lpf
            self.v_fwd = 0.0
            self.pub_vel.publish(Float32(data=float(self.v_fwd)))
            self.pub_dist.publish(Float32(data=float(self.dist)))
            self.last_time = now
            return

        # ---- Integrate with safety guards ----
        a_bc = a_clean - self.bias
        # guard against crazy spikes
        if abs(a_bc) > 30.0:   # > ~3g after comp? ignore as glitch
            a_bc = 0.0

        self.v_fwd += 0.5 * (a_bc + self.a_prev) * dt
        self.a_prev = a_bc

        # clamp to physical robot max speed
        self.v_fwd = clamp(self.v_fwd, -self.max_speed_mps, self.max_speed_mps)

        # distance integrate (signed or path length), then apply gain
        incr = (self.v_fwd * dt) if self.signed_distance else (abs(self.v_fwd) * dt)
        self.dist += self.distance_gain * incr

        # ---- Publish ----
        self.pub_vel.publish(Float32(data=float(self.v_fwd)))
        self.pub_dist.publish(Float32(data=float(self.dist)))

        # Debug occasionally
        if not hasattr(self, '_dbg_seen'):
            self._dbg_seen = 0
        self._dbg_seen += 1
        if self._dbg_seen % 50 == 0:
            self.get_logger().info(
                f"IMU msgs: {self._dbg_seen}, v={self.v_fwd:.3f} m/s, d={self.dist:.3f} m"
            )

        self.last_time = now


def main():
    rclpy.init()
    node = GyroDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
