import rclpy
from rclpy.node import Node

from turtleaudio.msg import StereoAudioBlock
import time
import numpy as np
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos = rclpy.qos.QoSProfile(depth=5,
                                   reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                   history=rclpy.qos.HistoryPolicy.KEEP_ALL)
        self.subscription = self.create_subscription(
            StereoAudioBlock,
            'audio_data',
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
        self.diff_tot = 0
        self.count_diff = 0
        self.count_frame = 0

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
        self.diff_tot += diff_time
        if self.diff_tot > (1024 / 48000):
            print(f"⚠️ Error: Time simulation diff is too high {self.diff_tot:.4f} seconds")
            self.count_diff += 1
            self.diff_tot = 0

        #=================================#
        # Number of messages received     #
        #=================================#

        self.count += 1
        print(f'Count: {self.count}')
        print(f'Real Count : {msg.header.frame_id}')
        #=================================#
        # Check if the data is None or 0  #
        #=================================#

        for data in msg.left.data:
            if data == 0 or data is None:
                #print("Error: left data is None")
                self.l_nan += 1
        for data in msg.right.data:
            if data == 0 or data is None:
                #print("Error: right data is None")
                self.r_nan += 1
        #print(f'Left None: {self.l_nan}, Right None: {self.r_nan}')

        #=================================#
        # check if the frame is the same  #
        #=================================#

        if np.int64(msg.header.frame_id) != np.int64(self.id_frame) + 1:
            print(f"⛔ Error: frame_id is not the same {msg.header.frame_id} != {np.int64(self.id_frame) + 1}")
            self.count_frame += np.int64(msg.header.frame_id) - np.int64(self.id_frame) - 1
        self.id_frame = msg.header.frame_id

        #=================================#
        # Determine the min and max time  #
        #=================================#

        temps = time.time() - self.current_time
        if temps > self.max_time and temps < 0.5:
            self.max_time = temps
        if temps < self.min_time and temps != 0:
            self.min_time = temps
        #print(f"Max Time: {self.max_time:.4f} seconds")
        #print(f"Min Time: {self.min_time:.4f} seconds")

        #=================================#
        # Determine the mean time         #
        #=================================#

        self.total_time += time.time() - self.current_time
        self.mean_time = self.total_time / self.count
        print(f'Mean Time: {self.mean_time:.4f} seconds')
        
        self.current_time = time.time()
        
        print(f"Nombre de frame perdu : {self.count_frame}")
        print(f"Nombre de diff tempo : {self.count_diff}")
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()