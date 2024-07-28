## This program is used to SAVE and PLOT the ground truth and odometry data from the robot.
## Need to run run SLAM_tolbox and the BAG file to get the data in parallel.
## The odom and gt data are saved in the 'gt_data.mat' file.
from lp_slam.src import ekf_funcs_lp as ekf
from matplotlib import pyplot as plt
import numpy as np
import pdb
import math
from scipy.io import savemat

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as rot
import rclpy.time
from tf2_ros import Buffer, TransformListener, Duration
from rclpy.time import Time

np.random.seed(10) # for reproducibility
plt.ion()

class get_gt(Node):
    def __init__(self, topic_name_odom):
        super().__init__('Pose_estimations') # name of the node
        self.del_l = 0 # robot linear displacement
        self.del_th = 0 # robot rotation
        self.prev_odom = np.zeros(3, dtype=np.float32)

        #**********************************************************************************************
        # TF2
        self.tf_buffer = Buffer(rclpy.time.Duration(seconds=1))
        # self.tf_timeout = Duration(seconds = 0.06, nanoseconds = 2000000)
        self.tf_timeout = Duration(seconds = 0.07)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        #**********************************************************************************************
        # Plot
        self.fig, self.ax = plt.subplots() 
        self.ax.set_aspect('equal', adjustable='box')
        self.fig.suptitle('Pose estimations')
        plt.ylabel("y [mm]")
        plt.xlabel("x [mm]")
        plt.grid()
        plt.show()

        self.odom_plt, = plt.plot([], [], linewidth=1, color='b', label='Odometry') # plot the odometry
        self.fig.canvas.draw() 
        self.gt_plt, = plt.plot([], [], linewidth=1, color='r', label='Ground Truth') # plot the ground truth
        self.fig.canvas.draw() 
        self.fig.canvas.flush_events() # update the plot
        
        self.odom_x_data, self.odom_y_data = [] , [] # odometry data
        self.gt_x_data, self.gt_y_data = [] , [] # ground truth data

        # self.xlims = [0, 450]
        # self.ylims = [-200, 200]

        self.id = 0 # index for the data.

        #**********************************************************************************************
        # Save data from the odometry
        self.subscription = self.create_subscription(
            Odometry,
            topic_name_odom,
            self.update_del_odom,
            1
        )
        self.subscription

        # self.subscription = self.create_subscription(

            

    # Update the plot of the odometry
    def update_plot_odom(self): 
        self.odom_plt.set_data(self.odom_x_data, self.odom_y_data)
        return self.ax
    
    # Update the ground truth plot
    def update_plot_gt(self):
        self.gt_plt.set_data(self.gt_x_data, self.gt_y_data)
        return self.ax

    #**********************************************************************************************
    
    # Callback functions odometry data
    def update_del_odom(self, msg):
        prev_x = self.prev_odom[0]
        prev_y = self.prev_odom[1]
        prev_theta = self.prev_odom[2]

        odo_x = msg.pose.pose.position.x * 1000
        odo_y = msg.pose.pose.position.y * 1000
        odo_theta = rot.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_euler('zyx')[0]

        del_x = odo_x - prev_x
        del_y = odo_y - prev_y
        del_l = del_x**2 + del_y**2
        if del_l >= 20 or abs(odo_theta - prev_theta) >= 2e-3:
            try:
                # now = Time.from_msg(msg.header.stamp)
                # now = self.get_clock().now().to_msg()
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
                
                
                # self.get_logger().info('Transform from map to base_link:')
                # self.get_logger().info(f'Translation: x={trans.transform.translation.x}, '
                #                     f'y={trans.transform.translation.y}, '
                #                     f'z={trans.transform.translation.z}')
                # self.get_logger().info(f'Rotation: x={trans.transform.rotation.x}, '
                #                     f'y={trans.transform.rotation.y}, '
                #                     f'z={trans.transform.rotation.z}, '
                #                     f'w={trans.transform.rotation.w}')

                self.prev_odom = [odo_x, odo_y, odo_theta]
                self.odom_x_data.append(odo_x)
                self.odom_y_data.append(odo_y)

                self.gt_x_data.append(trans.transform.translation.x * 1000)
                self.gt_y_data.append(trans.transform.translation.y * 1000)

                if self.id%1 == 0:
                    self.update_plot_odom()
                    self.update_plot_gt()
                    self.ax.relim()
                    self.ax.autoscale_view()
                    
                    # xlim = self.ax.get_xlim()
                    # ylim = self.ax.get_ylim()
                    self.ax.set_xlim(min(self.odom_x_data) - 2500, max(self.odom_x_data) + 2500)
                    self.ax.set_ylim(min(self.odom_y_data) - 2500, max(self.odom_y_data) + 2500)

                    self.fig.canvas.flush_events()
                    print('Updated plot...')
                self.id += 1
                
                print('id: ', self.id)

            except Exception as e:
                self.get_logger().warn(f'Could not transform: {e}')       


def main():
    rclpy.init()
    gt = get_gt('odom')
    try:
        rclpy.spin(gt)

    except KeyboardInterrupt:
        print('Keyboard interrupt. Saving data...')

    finally:
        gt_x  = gt.gt_x_data
        gt_y = gt.gt_y_data
        odom_x = gt.odom_x_data
        odom_y = gt.odom_y_data

        savemat('gt_data.mat', {'gt_x': gt_x, 'gt_y': gt_y, 'odom_x': odom_x, 'odom_y': odom_y})
        print('Data Saved!!!')
    
    pdb.set_trace()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
