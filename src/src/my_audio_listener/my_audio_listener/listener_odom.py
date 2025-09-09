import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import csv
import os


class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')

        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST
        )

        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            qos
        )

        # Fichier CSV
        self.csv_file = 'odom_data.csv'


    def listener_callback(self, msg: Odometry):
        # Extraire les données
        ori = msg.pose.pose.orientation
        yaw = np.arctan2(np.array([2*(ori.w*ori.z)]), np.array([1 - 2*(ori.z*ori.z)]))


        # Sauvegarde CSV
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(yaw)


def main(args=None):
    rclpy.init(args=args)
    odom_sub = OdomSubscriber()
    rclpy.spin(odom_sub)
    odom_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
