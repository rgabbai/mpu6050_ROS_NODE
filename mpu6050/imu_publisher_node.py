import rclpy
from sensor_msgs.msg import Imu  # IMU type msg topic 
import smbus                     #import SMBus module of I2C
from rclpy.node import Node

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#bus = smbus.SMBus(6) 	# I2C ch6 
#Device_Address = 0x68   # MPU6050 device address

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.imu_publisher = self.create_publisher(Imu, 'mpu6050/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # publish every 0.1 sec - Adjust the timer period as needed
        self.bus = smbus.SMBus(6)   # I2C ch6
        self.Device_Address = 0x68  # MPU6050 device address
        self.MPU_Init()

    def MPU_Init(self):
        #write to sample rate register
        self.bus.write_byte_data(self.Device_Address, SMPLRT_DIV, 6)

        #Write to power management register
        self.bus.write_byte_data(self.Device_Address, PWR_MGMT_1, 1)

        #Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, CONFIG, 0)

        #Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, GYRO_CONFIG, 24)

        #Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, INT_ENABLE, 1)



    def read_raw_data(self,addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value




    def publish_imu_data(self):
        imu_msg = Imu()
        # Your sensor reading and processing code here
        acc_x = self.read_raw_data(ACCEL_XOUT_H)
        acc_y = self.read_raw_data(ACCEL_YOUT_H)
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)

        #print("Read Gyroscope raw value")
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_YOUT_H)
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)

##      #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
##      
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        # Fill in the IMU message with the sensor data
        # Modify imu_msg.linear_acceleration and imu_msg.angular_velocity as needed
        imu_msg.linear_acceleration.x = Ax
        imu_msg.linear_acceleration.y = Ay
        imu_msg.linear_acceleration.z = Az
        imu_msg.angular_velocity.x = Gx
        imu_msg.angular_velocity.y = Gy
        imu_msg.angular_velocity.z = Gz
       # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az)
        # Publishing the IMU message
        self.imu_publisher.publish(imu_msg)
        #self.get_logger().info("IMU data published")




def main():
    rclpy.init()
    imu_publisher = ImuPublisherNode()
    print("MPU6050 publisher node activated");
    try:
        rclpy.spin(imu_publisher)
    
    except KeyboardInterrupt:
        pass
    
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

