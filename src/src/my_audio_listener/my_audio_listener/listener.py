import rclpy
from rclpy.node import Node

from turtleaudio.msg import StereoAudioBlock
import time
import numpy as np
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos = rclpy.qos.QoSProfile(depth=1,
                                   reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            StereoAudioBlock,
            'audio_sin',
            self.listener_callback,
            qos)
        
        self.subscription  # prevent unused variable warning
        self.count = 0
        self.l_nan = 0
        self.r_nan = 0
        self.start_time = 0
        self.total_time = 0
        self.mean_time = 0
        self.max_time = 0
        self.min_time = 1
        self.id_frame = 0 
        self.time_simu = 0
        self.start = True
        self.id_frame = 0
        self.current_time = 0

    def listener_callback(self, msg):

        if self.start:
            self.time_simu_start = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.start_time = time.time()
            self.current_time = time.time()
            self.start = False
        
        #=================================#
        # Time simulation diff            #
        #=================================#

        diff_time = (time.time() - self.start_time) - ((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.time_simu_start)
        print(f"Time Simu Diff : {diff_time:.4f} seconds")

        #=================================#
        # Number of messages received     #
        #=================================#

        self.count += 1
        print(f'Count: {self.count}')

        #=================================#
        # Check if the data is None or 0  #
        #=================================#

        for data in msg.left.data:
            if data == 0:
                #print("Error: left data is None")
                self.l_nan += 1
        for data in msg.right.data:
            if data == 0:
                #print("Error: right data is None")
                self.r_nan += 1
        print(f'Left None: {self.l_nan}, Right None: {self.r_nan}')

        #=================================#
        # check if the frame is the same  #
        #=================================#

        if np.int64(msg.header.frame_id) != np.int64(self.id_frame) + 1:
            print(f"Error: frame_id is not the same {msg.header.frame_id} != {np.int64(self.id_frame) + 1}")
        self.id_frame = msg.header.frame_id

        #=================================#
        # Determine the min and max time  #
        #=================================#

        temps = time.time() - self.current_time
        if temps > self.max_time and temps < 0.5:
            self.max_time = temps
        if temps < self.min_time and temps != 0:
            self.min_time = temps
        print(f"Max Time: {self.max_time:.4f} seconds")
        print(f"Min Time: {self.min_time:.4f} seconds")

        #=================================#
        # Determine the mean time         #
        #=================================#

        self.total_time += time.time() - self.current_time
        self.mean_time = self.total_time / self.count
        print(f'Mean Time: {self.mean_time:.4f} seconds')
        
        self.current_time = time.time()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()