#!/usr/bin/env python3

import argparse
from collections import deque
from math import sqrt

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuLinearAccelerationPlotter(Node):
    def __init__(self, topic_name: str, history_size: int):
        super().__init__('imu_linear_acceleration_plotter')

        self.topic_name = topic_name
        self.history_size = history_size

        self.sample_index = 0
        self.indices = deque(maxlen=history_size)
        self.acc_x = deque(maxlen=history_size)
        self.acc_y = deque(maxlen=history_size)
        self.acc_z = deque(maxlen=history_size)
        self.acc_norm = deque(maxlen=history_size)

        self.subscription = self.create_subscription(
            Imu,
            topic_name,
            self.imu_callback,
            10,
        )

        plt.ion()
        self.figure, self.axes = plt.subplots(figsize=(10, 6))
        self.line_x, = self.axes.plot([], [], label='linear_acceleration.x')
        self.line_y, = self.axes.plot([], [], label='linear_acceleration.y')
        self.line_z, = self.axes.plot([], [], label='linear_acceleration.z')
        self.line_norm, = self.axes.plot([], [], label='|linear_acceleration|', linewidth=2)

        self.axes.set_title(f'IMU Linear Acceleration Debug Plot: {topic_name}')
        self.axes.set_xlabel('Sample')
        self.axes.set_ylabel('Acceleration (m/s^2)')
        self.axes.grid(True, linestyle='--', alpha=0.4)
        self.axes.legend(loc='upper right')
        self.figure.tight_layout()

        self.get_logger().info(f'Subscribed to {topic_name}')
        self.get_logger().info('Close the plot window to stop the node.')

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.indices.append(self.sample_index)
        self.acc_x.append(ax)
        self.acc_y.append(ay)
        self.acc_z.append(az)
        self.acc_norm.append(sqrt(ax * ax + ay * ay + az * az))
        self.sample_index += 1

    def update_plot(self):
        if not self.indices:
            return

        x_values = list(self.indices)
        self.line_x.set_data(x_values, list(self.acc_x))
        self.line_y.set_data(x_values, list(self.acc_y))
        self.line_z.set_data(x_values, list(self.acc_z))
        self.line_norm.set_data(x_values, list(self.acc_norm))

        self.axes.relim()
        self.axes.autoscale_view()
        self.axes.set_xlim(x_values[0], max(x_values[-1], x_values[0] + 1))

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()


def main():
    parser = argparse.ArgumentParser(description='Plot IMU linear acceleration from a ROS 2 topic.')
    parser.add_argument(
        '--topic',
        default='/spot/imu_sensor_broadcaster/imu',
        help='IMU topic to subscribe to',
    )
    parser.add_argument(
        '--history',
        type=int,
        default=500,
        help='Number of recent samples to keep in the plot',
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=20.0,
        help='Plot refresh rate in Hz',
    )
    args = parser.parse_args()

    rclpy.init()
    node = ImuLinearAccelerationPlotter(args.topic, args.history)

    try:
        while rclpy.ok() and plt.fignum_exists(node.figure.number):
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
            plt.pause(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()