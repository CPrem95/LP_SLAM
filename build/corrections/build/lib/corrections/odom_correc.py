import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# class OdomCorrect(Node):
#     def __init__(self):
#         super().__init__('odom_correct')
#         self.odom_sub = Subscriber(self, Odometry, '/odom')
#         self.scan_sub = Subscriber(self, LaserScan, '/scan')
#         self.ts = TimeSynchronizer([self.odom_sub, self.scan_sub], 10)
#         self.ts = ApproximateTimeSynchronizer([self.odom_sub, self.scan_sub], 10, 0.1)
#         self.ts.registerCallback(self.callback)

#     def callback(self, odom_msg, laser_msg):
#         self.get_logger().info(f'odom: {odom_msg.header.stamp.sec}.{odom_msg.header.stamp.nanosec}, laser: {laser_msg.header.stamp.sec}.{laser_msg.header.stamp.nanosec}')

# def main(args=None):
#     rclpy.init(args=args)
#     odom_correct = OdomCorrect()
#     rclpy.spin(odom_correct)
#     odom_correct.destroy_node()
#     rclpy.shutdown()

class OdomCorrect(Node):
    def __init__(self):
        super().__init__('odom_correct')
        self.scan_data = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_listener_callback,
            1)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan2',
            self.scan_listener_callback,
            1)
        
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10)
        
    def scan_listener_callback(self, msg):
        self.scan_data = msg

    def odom_listener_callback(self, msg):
        if self.scan_data is not None:
            self.scan_data.header.stamp = msg.header.stamp
            self.scan_pub.publish(self.scan_data)

def main(args=None):
    rclpy.init(args=args)
    odom_correct = OdomCorrect()
    rclpy.spin(odom_correct)
    odom_correct.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
