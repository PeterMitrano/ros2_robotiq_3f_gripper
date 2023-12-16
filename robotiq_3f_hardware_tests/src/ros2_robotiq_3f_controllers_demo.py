#! /usr/bin/env python
from time import sleep

import rclpy
import numpy as np
from std_msgs.msg import Float64MultiArray


def main():
    rclpy.init()
    node = rclpy.create_node('robotiq_3f_controllers_demo')

    pub = node.create_publisher(Float64MultiArray, 'individual_position_controller/commands', 10)

    msg = Float64MultiArray()
    msg.data = [0.0, 0.0, 0.0, 0.0]
    v = 0.5
    dt = 0.5

    # Move each finger sinusoidally with different frequencies
    for i in np.linspace(0, 8 * np.pi, 100):
        msg.data[0] = 0.5 - np.cos(v * 0.5 * i) / 2
        msg.data[1] = 0.5 - np.cos(v * 1 * i) / 2
        msg.data[2] = 0.5 - np.cos(v * 2 * i) / 2
        msg.data[3] = 0.5 - np.sin(v * 0.2 * i) / 2

        print(msg.data)
        pub.publish(msg)

        sleep(dt)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
