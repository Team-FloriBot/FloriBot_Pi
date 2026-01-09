#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import time
import matplotlib.pyplot as plt

class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test_node')
        #qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Publisher / Subscriber
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Daten für Plot
        self.x_data = []
        self.y_data = []
        self.q_data = []  # Quaternion für Yaw

        self.get_logger().info('MotionTestNode gestartet')

    # Odom Callback
    def odom_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)
        self.q_data.append(msg.pose.pose.orientation)

    # Helper: sicheren Twist erzeugen
    def _make_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        twist = Twist()
        twist.linear.x = float(lx)
        twist.linear.y = float(ly)
        twist.linear.z = float(lz)
        twist.angular.x = float(ax)
        twist.angular.y = float(ay)
        twist.angular.z = float(az)
        return twist

    # Helper: Yaw aus Quaternion
    def get_yaw(self):
        if not self.q_data:
            return 0.0
        q = self.q_data[-1]
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    # Geradeausfahrt odom-basiert
    def drive_straight(self, distance, v):
        if not self.x_data:
            self.get_logger().info('Keine Odom-Daten verfügbar!')
            return
        start_x = self.x_data[-1]
        start_y = self.y_data[-1]
        twist = self._make_twist(lx=v)

        self.get_logger().info(f'Geradeausfahrt startet: {distance} m')
        rate_hz = 10
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)  # sorgt dafür, dass Odom aktualisiert wird
            current_x = self.x_data[-1]
            current_y = self.y_data[-1]
            traveled = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            self.get_logger().info(f'Zurückgelegt: {traveled} m')
            if traveled >= distance:
                break
            self.pub.publish(twist)
            time.sleep(1/rate_hz)


        self.pub.publish(self._make_twist())  # Stop
        self.get_logger().info('Geradeausfahrt beendet')
        self._plot_path('Geradeausfahrt')

    # Rotation auf der Stelle odom-basiert
    def rotate(self, n_rotations, omega):
        if not self.q_data:
            self.get_logger().info('Keine Odom-Daten verfügbar!')
            return
        start_yaw = self.get_yaw()
        target_angle = 2*math.pi*n_rotations
        twist = self._make_twist(az=omega)

        accumulated = 0.0
        last_yaw = start_yaw
        rate_hz = 10
        self.get_logger().info(f'Rotation startet: {n_rotations}x360°')

        while rclpy.ok() and accumulated < target_angle:
            rclpy.spin_once(self, timeout_sec=0.01)  # sorgt dafür, dass Odom aktualisiert wird
            current_yaw = self.get_yaw()
            delta = current_yaw - last_yaw
            if delta > math.pi:
                delta -= 2*math.pi
            elif delta < -math.pi:
                delta += 2*math.pi
            accumulated += abs(delta)
            self.get_logger().info(f'Rotated: {accumulated}')
            last_yaw = current_yaw
            self.pub.publish(twist)
            time.sleep(1/rate_hz)

        self.pub.publish(self._make_twist())  # Stop
        self.get_logger().info('Rotation beendet')
        self._plot_path(f'{n_rotations}x Rotation')

    # Kreisfahrt (über v, omega, odom-basiert optional)
    def drive_circle(self, radius, v, n_circles=1):
        if not self.q_data:
            self.get_logger().info('Keine Odom-Daten verfügbar!')
            return

        omega = v / radius
        twist = self._make_twist(lx=v, az=omega)

        start_yaw = self.get_yaw()
        last_yaw = start_yaw
        accumulated = 0.0
        target_angle = 2 * math.pi * n_circles

        self.get_logger().info(
            f'Kreisfahrt startet: Radius={radius} m, v={v} m/s, Umdrehungen={n_circles}'
        )

        rate_hz = 10
        while rclpy.ok() and accumulated < target_angle:
            rclpy.spin_once(self, timeout_sec=0.01)

            current_yaw = self.get_yaw()
            delta = current_yaw - last_yaw

            # Winkel normalisieren auf [-pi, pi]
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi

            accumulated += abs(delta)
            last_yaw = current_yaw

            self.get_logger().info(
                f'Yaw akkumuliert: {accumulated:.3f} / {target_angle:.3f} rad'
            )

            self.pub.publish(twist)
            time.sleep(1 / rate_hz)

        self.pub.publish(self._make_twist())  # Stop
        self.get_logger().info('Kreisfahrt beendet')
        self._plot_path('Kreisfahrt (odom-basiert)')


    # Plot der bisherigen X/Y Daten
    def _plot_path(self, title='Pfad'):
        plt.figure()
        plt.plot(self.x_data, self.y_data, marker='o')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title(title)
        plt.axis('equal')
        plt.grid(True)
        plt.show()

    def wait_for_odom(self, timeout=5.0):
        """Warte auf erste Odom-Nachricht"""
        start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            if self.x_data and self.y_data:
                return True
            if self.get_clock().now().nanoseconds / 1e9 - start_time > timeout:
                self.get_logger().warning('Timeout: keine Odom-Nachricht empfangen!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)



def main():
    rclpy.init()
    node = MotionTestNode()

    if not node.wait_for_odom():
        exit(1)  # Abbruch, wenn keine Odom-Nachricht



    # Kurze Verzögerung, damit Odom initialisiert
    rclpy.spin_once(node, timeout_sec=1.0)

    # Testläufe
    #node.drive_circle(radius=0.5, v=1.8)
    node.drive_straight(distance=3.0, v=12.2)
    #node.rotate(n_rotations=1, omega=0.5)
    #node.rotate(n_rotations=5, omega=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
