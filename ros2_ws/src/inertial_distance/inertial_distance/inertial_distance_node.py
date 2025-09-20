#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster  # keeps TF publishing
from rclpy.qos import qos_profile_sensor_data


def clamp(v, lo, hi): return max(lo, min(hi, v))

def quaternion_from_euler(roll, pitch, yaw):
    """Compute quaternion (x,y,z,w) from ZYX euler (roll, pitch, yaw)."""
    cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    w = cr*cp*cy + sr*sp*sy
    return (x, y, z, w)

class InertialDistance(Node):
    """
    Estimate travelled distance by integrating accelerometer along robot forward axis.
    Orientation (yaw) from gyro; roll/pitch stabilized via accel.
    Optional ZUPT using wheel RPM and small encoder-blend to reduce drift.
    """

    def __init__(self):
        super().__init__('inertial_distance')

        # ---- Parameters ----
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('rpm_topic', '/wheel_rpm')
        self.declare_parameter('use_rpm_for_zupt', True)
        self.declare_parameter('zupt_vel_epsilon', 0.02)
        self.declare_parameter('zupt_hold_ms', 150)
        self.declare_parameter('beta_enc_blend', 0.05)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('distance_topic', '/inertial_distance')
        self.declare_parameter('odom_topic', '/odom_inertial')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.imu_topic = self.get_parameter('imu_topic').value
        self.rpm_topic = self.get_parameter('rpm_topic').value
        self.use_rpm_for_zupt = bool(self.get_parameter('use_rpm_for_zupt').value)
        self.zupt_eps = float(self.get_parameter('zupt_vel_epsilon').value)
        self.zupt_hold_ms = int(self.get_parameter('zupt_hold_ms').value)
        self.beta = float(self.get_parameter('beta_enc_blend').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.distance_topic = self.get_parameter('distance_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # ---- Subscribers ----
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos)
        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data)

        self.sub_rpm = self.create_subscription(Float32MultiArray, self.rpm_topic, self._rpm_cb, qos)

        # ---- Publishers ----
        self.pub_dist = self.create_publisher(Float32, self.distance_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- State ----
        self.last_time = self.get_clock().now()
        self.has_imu = False
        self.ax = self.ay = self.az = 0.0
        self.gz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.ax_prev = 0.0
        self.vx_acc = 0.0
        self.vx_enc = 0.0
        self.x = self.y = self.dist = 0.0
        self.still_since = None
        self.ax_bias = 0.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self._update)

        self.get_logger().info(
            f"inertial_distance: hz={self.rate_hz} imu={self.imu_topic} rpm={self.rpm_topic} beta={self.beta}")

    # ---- Callbacks ----
    def _imu_cb(self, msg: Imu):
        self.gz = float(msg.angular_velocity.z)
        self.ax = float(msg.linear_acceleration.x)
        self.ay = float(msg.linear_acceleration.y)
        self.az = float(msg.linear_acceleration.z)
        self.has_imu = True

        # roll/pitch from accel (low-gain complementary)
        g = math.sqrt(self.ax*self.ax + self.ay*self.ay + self.az*self.az) + 1e-6
        axn, ayn, azn = self.ax/g, self.ay/g, self.az/g
        pitch_meas = math.asin(-clamp(axn, -1.0, 1.0))
        roll_meas  = math.atan2(ayn, azn)
        k = 0.02
        self.pitch = (1.0 - k) * self.pitch + k * pitch_meas
        self.roll  = (1.0 - k) * self.roll  + k * roll_meas

    def _rpm_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            rpmL, rpmR = float(msg.data[0]), float(msg.data[1])
            wL = rpmL * 2.0*math.pi / 60.0
            wR = rpmR * 2.0*math.pi / 60.0
            self.vx_enc = self.r * 0.5 * (wL + wR)

    # ---- Core loop ----
    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.2 or not self.has_imu:
            self.last_time = now
            return

        # integrate yaw from gyro
        self.yaw += self.gz * dt

        # rotate body accel -> world-x using ZYX
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        axw = cy*cp*self.ax + (cy*sp*sr - sy*cr)*self.ay + (cy*sp*cr + sy*sr)*self.az

        # ZUPT (if wheels basically stopped, learn bias and clamp vx)
        stopped = False
        if self.use_rpm_for_zupt:
            if abs(self.vx_enc) < self.zupt_eps:
                if self.still_since is None:
                    self.still_since = now
                elif (now - self.still_since) >= Duration(seconds=self.zupt_hold_ms/1000.0):
                    stopped = True
            else:
                self.still_since = None
        if stopped:
            self.ax_bias = 0.98*self.ax_bias + 0.02*axw
            self.vx_acc = 0.0

        # integrate accel->vel (trapz) with learned bias removed
        axw_nog = axw - self.ax_bias
        self.vx_acc += 0.5 * (axw_nog + self.ax_prev) * dt
        self.ax_prev = axw_nog

        # mild blend toward encoder velocity to bound drift
        vx_fused = (1.0 - self.beta) * self.vx_acc + self.beta * self.vx_enc

        # integrate distance and x,y (for viz)
        self.dist += abs(vx_fused) * dt
        self.x += vx_fused * math.cos(self.yaw) * dt
        self.y += vx_fused * math.sin(self.yaw) * dt

        # publish distance
        dmsg = Float32(); dmsg.data = float(self.dist)
        self.pub_dist.publish(dmsg)

        # publish inertial odom + TF
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx_fused
        odom.twist.twist.angular.z = self.gz
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        self.last_time = now

def main():
    rclpy.init()
    node = InertialDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
