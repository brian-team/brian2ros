import rclpy
from rclpy.node import Node

from turtleaudio.msg import StereoAudioBlock
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            StereoAudioBlock,
            'audio_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.count = 0
        self.l_nan = 0
        self.r_nan = 0
        self.start_time = time.time()
        self.total_time = 0
        self.mean_time = 0
        self.max_time = 0
        self.min_time = 1
        self.id_frame = 0 
    def listener_callback(self, msg):
        self.count += 1
        # for data in msg.left.data:
        #     if data is None:
        #         print("Error: left data is None")
        #         self.l_nan += 1
        # for data in msg.right.data:
        #     if data == 0 or data is None:
        #         print("Error: right data is None")
        #         self.r_nan += 1
        msg.header.frame_id
        self.total_time += time.time() - self.start_time
        temps = time.time() - self.start_time
        if temps > self.max_time and temps < 0.5:
            self.max_time = temps
        if temps < self.min_time and temps != 0:
            self.min_time = temps
        print(f"Time: {time.time() - self.start_time:.4f} seconds")
        print(f"Max Time: {self.max_time:.4f} seconds")
        print(f"Min Time: {self.min_time:.4f} seconds")
        self.start_time = time.time()
        self.mean_time = self.total_time / self.count
        print(f'Left: {len(msg.left.data)}, Right: {len(msg.right.data)}')
        print(f'Count: {self.count}')
        print(f'Mean Time: {self.mean_time:.4f} seconds')
        
        #print(f'Left None: {self.l_nan}, Right None: {self.r_nan}')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()