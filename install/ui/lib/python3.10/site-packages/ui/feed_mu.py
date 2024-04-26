import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class FeedMuPublisher(Node):
    def __init__(self):
        super().__init__('feed_mu')

        self.publisher_ = self.create_publisher(Float32MultiArray, 'est_mu', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.i, self.i + 1, self.i + 2, 100, 200, 200, 150, 50, 300, 500, 700, 800, 900] 
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    feed_mu_publisher = FeedMuPublisher()
    rclpy.spin(feed_mu_publisher)
    feed_mu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()