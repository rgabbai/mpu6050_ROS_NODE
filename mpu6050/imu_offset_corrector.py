import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import transforms3d  # New library for quaternion handling
import numpy as np
import math  

class ImuOffsetCorrector(Node):
    def __init__(self):
        super().__init__('imu_offset_corrector')
        
        # Parameters for offset ~ defualt location
        self.declare_parameter('offset_x', -0.24)   # X: -24.0 cm
        self.declare_parameter('offset_y', -0.105)  # Y: -10.5 cm
        self.declare_parameter('offset_z', 0.14)    # Z: +14.0 cm 

        self.offset = np.array([
            self.get_parameter('offset_x').get_parameter_value().double_value,
            self.get_parameter('offset_y').get_parameter_value().double_value,
            self.get_parameter('offset_z').get_parameter_value().double_value
        ])
        
        # Subscriber to the filtered IMU topic by imu_filter_madgwick
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',          
            self.imu_callback,
            10
        )
        
        # Publisher for corrected IMU data
        self.publisher = self.create_publisher(Imu, '/imu/data_corrected', 10)
        
        # Publisher for roll, pitch, yaw values
        self.rpy_publisher = self.create_publisher(Vector3, '/imu/rpy', 10)

    def imu_callback(self, msg):
        # Extract orientation from the IMU message (quaternion)
        orientation_quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
       

        # Compute the offset-induced linear velocity due to angular velocity (cross-product)
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        induced_velocity = np.cross(angular_velocity, self.offset)
        
        # Apply correction to orientation based on induced motion
        # In this case, we assume correction only affects orientation. Depending on the robot's setup,
        # you may need more advanced corrections that involve combining rotation and induced translation.
        
        # Corrected orientation remains as it is for simplicity (but could involve matrix adjustments)
        corrected_msg = Imu()
        corrected_msg.header = msg.header
        corrected_msg.orientation = msg.orientation  # Keep the same orientation (for simplicity)
        corrected_msg.angular_velocity = msg.angular_velocity  # Angular velocity unchanged

        # If you need to compensate or correct linear acceleration based on induced motion:
        corrected_msg.linear_acceleration.x = msg.linear_acceleration.x - induced_velocity[0]
        corrected_msg.linear_acceleration.y = msg.linear_acceleration.y - induced_velocity[1]
        corrected_msg.linear_acceleration.z = msg.linear_acceleration.z - induced_velocity[2]
        
        # Convert corrected quaternion to roll, pitch, and yaw using transforms3d
        corrected_orientation_quat = [
            corrected_msg.orientation.w,
            corrected_msg.orientation.x,
            corrected_msg.orientation.y,
            corrected_msg.orientation.z,
        ]
        
        roll, pitch, yaw = transforms3d.euler.quat2euler(corrected_orientation_quat)
        
        # Convert roll, pitch, and yaw to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        # Convert yaw to the 0-360 degree range using the new function
        yaw_deg = self.convert_to_360(yaw_deg)

        # Create and publish Vector3 message with roll, pitch, and yaw
        rpy_msg = Vector3()
        rpy_msg.x = roll_deg
        rpy_msg.y = pitch_deg
        rpy_msg.z = yaw_deg
        self.rpy_publisher.publish(rpy_msg)
        
        # Publish the corrected IMU data
        self.publisher.publish(corrected_msg)
    
    @staticmethod
    def convert_to_360(yaw_angle: float) -> float:
        """Converts a yaw angle to a 0-360 degree range."""
        if yaw_angle < 0.0:
            return yaw_angle + 360.0
        else:
            return yaw_angle

def main(args=None):
    rclpy.init(args=args)
    node = ImuOffsetCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
