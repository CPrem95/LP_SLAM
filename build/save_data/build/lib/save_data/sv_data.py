# This script saves the data from the radars, trilateration and odometry to a .mat file
# Has to run the ros BAG file and the trilaterator in parallel.
# The data is saved in the 'dataset.mat' file.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
from scipy.io import savemat
from scipy.spatial.transform import Rotation as rot
import pdb

class SaveData(Node):
    def __init__(self, topic_name_r1, topic_name_r2, topic_name_r3, topic_name_r4, topic_name_tri_L, topic_name_tri_R, topic_name_odom):
        super().__init__('SaveData')
        # Declare parameters
        self.declare_parameter('ds_name', 'dataset')
        
        # Get the parameter
        self.ds_name = self.get_parameter('ds_name').get_parameter_value().string_value

        n_data = 30000
        len_data = 500
        self.data_r1 = np.zeros((n_data, len_data), dtype=np.float32)
        self.data_r2 = np.zeros((n_data, len_data), dtype=np.float32)
        self.data_r3 = np.zeros((n_data, len_data), dtype=np.float32)
        self.data_r4 = np.zeros((n_data, len_data), dtype=np.float32)

        self.data_tri_L = np.zeros((n_data, 10), dtype=np.float32)
        self.data_tri_R = np.zeros((n_data, 10), dtype=np.float32)

        self.data_odom = np.zeros((n_data, 3), dtype=np.float32)

        self.id = 1 # index for the data. = 1 because the first data is zero for all arrays
        self.prev_odom = np.zeros(3, dtype=np.float32)

        #**********************************************************************************************
        # Save data from the odometry
        self.subscription = self.create_subscription(
            Odometry,
            topic_name_odom,
            self.save_obs_odom,
            1
        )
        self.subscription

        #**********************************************************************************************
        # Save raw data from the radars
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_r1,
            self.save_obs_r1,
            1
        )
        self.subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_r2,
            self.save_obs_r2,
            1
        )
        self.subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_r3,
            self.save_obs_r3,
            1
        )
        self.subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_r4,
            self.save_obs_r4,
            1
        )
        self.subscription

        #**********************************************************************************************
        # Save data from the trilateration
        self.subscription = self.create_subscription(
            Float32MultiArray,
            topic_name_tri_L,
            self.save_obs_tri_L,
            1
        )
        self.subscription
        self.subscription = self.create_subscription(

            Float32MultiArray,
            topic_name_tri_R,
            self.save_obs_tri_R,
            1
        )
        self.subscription        

    ############################################################################################################
    # Callback functions
    ############################################################################################################

    #**********************************************************************************************
    # Callback functions raw data from the radars
    def save_obs_r1(self, msg):
        self.data_r1[self.id] = msg.data
        # print(self.data)
    def save_obs_r2(self, msg):
        self.data_r2[self.id] = msg.data
        # print(self.data)
    def save_obs_r3(self, msg):
        self.data_r3[self.id] = msg.data
        # print(self.data)
    def save_obs_r4(self, msg):
        self.data_r4[self.id] = msg.data
        # print(self.data)
    
    #**********************************************************************************************
    # Callback functions trilateration data
    def save_obs_tri_L(self, msg):
        # print(msg.data)
        # print(type(msg.data))
        # print(len(msg.data))
        # print('\n')
        self.data_tri_L[self.id, 0:len(msg.data)] = msg.data
        # print(self.data)
    def save_obs_tri_R(self, msg):
        self.data_tri_R[self.id, 0:len(msg.data)] = msg.data
        # print(self.data)

    #**********************************************************************************************
    # Callback functions odometry data
    def save_obs_odom(self, msg):
        prev_x = self.prev_odom[0]
        prev_y = self.prev_odom[1]
        prev_theta = self.prev_odom[2]

        cur_x = msg.pose.pose.position.x *1000
        cur_y = msg.pose.pose.position.y *1000
        cur_theta = rot.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]).as_euler('zyx')[0]
        # euler_angles = rotation.as_euler('xyz', degrees=False) 
        # cur_theta = euler_angles[2]

        del_x = cur_x - prev_x
        del_y = cur_y - prev_y
        del_l = del_x**2 + del_y**2
        if del_l >= 20 or abs(cur_theta - prev_theta) >= 2e-3:
        # if abs(cur_x - prev_x) > 0.01 or abs(cur_y - prev_y) > 0.01 or abs(cur_theta - prev_theta) > 0.01:
            self.data_odom[self.id] = [cur_x, cur_y, cur_theta]
            print('id: ', self.id)
            self.id += 1
            self.prev_odom = [cur_x, cur_y, cur_theta]
            # print(self.prev_odom)
            # print(del_l)

############################################################################################################
# Main function
############################################################################################################
def main(args=None):
    print('\nCollecting data...')
    rclpy.init(args=args)
    save_data = SaveData('UWBradar1/readings', 'UWBradar2/readings', 'UWBradar3/readings', 'UWBradar4/readings', 'LeftObs/range_bear', 'RightObs/range_bear', 'odom')
    
    try:
        rclpy.spin(save_data)

    except KeyboardInterrupt:
        print('Keyboard interrupt. Saving data...')

    finally:
        #**********************************************************************************************
        # Save the data to a .mat file
        # # Create a dictionary to hold the arrays
        # data_dict = {
        #     'data_r1': save_data.data_r1,
        #     'data_r2': save_data.data_r2,
        #     'data_r3': save_data.data_r3,
        #     'data_r4': save_data.data_r4,
        #     'data_tri_L': save_data.data_tri_L,
        #     'data_tri_R': save_data.data_tri_R,
        #     'data_odom': save_data.data_odom,
        #     'id': save_data.id
        # }
        # # Save the dictionary to a .mat file
        # savemat('data.mat', data_dict)

        obsLHS = np.zeros((save_data.id, 5, 2))
        obsRHS = np.zeros((save_data.id, 5, 2))

        for i in range(save_data.id):
            obsLHS[i, :, :] = np.reshape(save_data.data_tri_L[i, :], (1, 5, 2))
            obsRHS[i, :, :] = np.reshape(save_data.data_tri_R[i, :], (1, 5, 2))

        R1 = save_data.data_r1[:save_data.id, :]
        R2 = save_data.data_r2[:save_data.id, :]
        R3 = save_data.data_r3[:save_data.id, :]
        R4 = save_data.data_r4[:save_data.id, :]

        odom = save_data.data_odom[:save_data.id, :]

        savemat(str(save_data.ds_name) +".mat", {"odom": odom, "R1": R1, "R2": R2, "R3": R3, "R4": R4, "obsLHS": obsLHS, "obsRHS": obsRHS})

        save_data.destroy_node()
        rclpy.shutdown()

    