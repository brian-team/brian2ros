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
        print("Node 'odom_subscriber' started, listening to 'odom' topic.")
        # Fichier CSV
        self.csv_file = 'odom_data.csv'
        self.last_yaw = None

    def listener_callback(self, msg: Odometry):
        # Extraire les données
        ori = msg.pose.pose.orientation
        yaw = np.arctan2(np.array([2*(ori.w*ori.z)]), np.array([1 - 2*(ori.z*ori.z)])) * 180/np.pi
        #if yaw == self.last_yaw:
            # self.get_logger().info('Yaw inchangé, arrêt de l\'écoute.')
            # self.destroy_node()
            # rclpy.shutdown()
        self.last_yaw = yaw

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
