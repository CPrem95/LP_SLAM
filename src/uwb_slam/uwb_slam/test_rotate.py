import numpy as np
from scipy.optimize import least_squares
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import uwb_slam.func_LM as LM
import math
from tf_transformations import euler_from_quaternion
import pickle

class MultiSubscriber(Node):
    def __init__(self):
        self.ini_anc = [0, 0, 0] # Whether the anchor is initialized
        self.position = 0 # robot position
        self.yaw = 0 # robot orientation
        self.linear_vel = 0 # robot linear velocity
        self.angular_vel = 0 # robot angular velocity
        self.n_data = 500 # number of data points for LM
        self.odom_x = np.zeros(self.n_data) # store the robot x odometry position
        self.odom_y = np.zeros(self.n_data) # store the robot y odometry position
        self.odom_th = np.zeros(self.n_data) # store the robot theta odometry position
        self.ranges = np.zeros(self.n_data) # store the range readings
    
        self.count1 = 0 # count the number of range readings
        self.anc_1_reading = 10000 # range reading from anchor 1
        self.anc_2_reading = 0 # range reading from anchor 2
        self.anc_3_reading = 0 # range reading from anchor 3

        self.prev_yaw = 0 # store the previous orientation
        
        self.robot_r = 250 # robot radius [mm]

        super().__init__('multiple_subscriber')
        # subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            'odom',  # Replace 'odom_topic' with the actual topic name
            self.odometry_callback,
            10  # Adjust the queue size as needed
        )
        self.subscription  # prevent unused variable warning
        # subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Replace 'cmd_vel' with the actual topic name
            self.cmd_vel_callback,
            10  # Adjust the queue size as needed
        )
        self.subscription  # prevent unused variable warning
        # subscribe to uwb range topic <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        self.subscription = self.create_subscription(
            Int16,
            'ranges/value1',  # Replace 'anc_1' with the actual topic name
            self.anc_1_callback,
            10  # Adjust the queue size as needed
        )

    def odometry_callback(self, msg):
        # Your callback function logic here
        # Access odometry data using msg object
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    def anc_1_callback(self, msg):
        self.anc_1_reading = int(msg.data)
        # print(self.anc_1_reading)

    def cmd_vel_callback(self, msg):
        # Your callback function logic here
        # Access linear and angular velocities using msg object
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        # print("Received cmd_vel:\nLinear Velocity: %.4f\nAngular Velocity: %.4f" % (self.linear_vel, self.angular_vel))
        if self.ini_anc[0] == 0 and 0 < self.anc_1_reading < 12000:
            if self.linear_vel == 0 and self.angular_vel != 0 and self.count1 < self.n_data:
                if abs(self.yaw - self.prev_yaw) > 0.01: # 3.43 degrees
                    self.ranges[self.count1] = self.anc_1_reading # save the range reading
                    self.odom_x[self.count1] = self.position.x * 1000 # save the robot x odometry position
                    self.odom_y[self.count1] = self.position.y * 1000 # save the robot y odometry position
                    self.odom_th[self.count1] = self.yaw # save the robot theta odometry position

                    self.count1 += 1
                    print("Count1: ", self.count1, " anc_1 reading: ", self.anc_1_reading)
                    self.prev_yaw = self.yaw
            elif self.count1 == self.n_data:
                x_r = self.robot_r*np.cos(self.odom_th) + self.odom_x # tag x positions
                y_r = self.robot_r*np.sin(self.odom_th) + self.odom_y # tag y positions

                # mean_r = np.mean(self.ranges)
                # std_r = np.std(self.ranges)

                # x_r = x_r[self.ranges < mean_r + 2*std_r]
                # x_r = x_r[self.ranges > mean_r - 2*std_r]

                # y_r = y_r[self.ranges < mean_r + 2*std_r]
                # y_r = y_r[self.ranges > mean_r - 2*std_r]

                # R_r = self.ranges[self.ranges < mean_r + 2*std_r]
                # R_r = self.ranges[self.ranges > mean_r - 2*std_r]
                
                # print("x_r: ", x_r)
                # print("y_r: ", y_r)
                # print("length x_r", len(x_r))
                # print("length y_r", len(y_r))
                # print("length R_r", len(R_r))

                # find anchor 1 position
                anc_1_pos = LM.find_anchor(x_r, y_r, self.ranges)
                print("Anchor position: ", anc_1_pos.x)

                # obj0, obj1, obj2 are created here...

                # Saving the objects:
                # f = open('objs_1.pkl', 'w')  # Python 3: open(..., 'wb')
                # pickle.dump([self.ranges, self.odom_th, self.odom_x, self.odom_y], f)
                # f.close()

                # # Getting back the objects:
                # with open('objs.pkl') as f:  # Python 3: open(..., 'rb')
                #     obj0, obj1, obj2 = pickle.load(f)

                self.ini_anc[0] = 1
                self.count1 = 0

                # print("Initial Anchor: ", self.ini_anc)
        # reset the count1
        elif self.linear_vel != 0 and self.angular_vel == 0:
            self.count1 = 0        # print(f"Received Odometry:\nPosition: {self.position}")
   
######################################################################
# main

# print the result
def main(args=None):
    rclpy.init(args=args)

    multiple_subscribers_node = MultiSubscriber()

    try:
        rclpy.spin(multiple_subscribers_node)
    except KeyboardInterrupt:
        pass

    multiple_subscribers_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
