import rclpy
from sensor_msgs.msg import Imu  # IMU type msg topic 
import smbus                     #import SMBus module of I2C
from rclpy.node import Node
#from std_srvs.srv import Trigger, Trigger_Response
from std_srvs.srv import Trigger
from time import sleep  


# MPU650 
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

ACC_FACTOR   = 16384.0     # 2g
GYRO_FACTOR  = 131.0       # +/-250
TEMP_A       = 340.0       # C = temp_raw/TEMP_A+TEMP_B
TEMP_B       = 36.53

# Calibration constants
SAMPLES      = 2000  # calibration reads
TIME_BTWEEN_SAMPLE = 0.001  # 1ms
LPF_A        = 0.1   # Low pass filter alfa 


#bus = smbus.SMBus(6) 	# I2C ch6 
#Device_Address = 0x68   # MPU6050 device address

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.imu_publisher = self.create_publisher(Imu, 'mpu6050/imu/data', 10)
        #self.calibration_service = self.create_service(Trigger, 'calibrate_imu', self.handle_calibration_request)
        self.calibration_service = self.create_service(Trigger, 'calibrate_imu', self.handle_calibration_request)
        self.bus = smbus.SMBus(6)   # I2C ch6
        self.Device_Address = 0x68  # MPU6050 device address
        self.MPU_Init()

        self.acc_x_avg = 0.0
        self.acc_y_avg = 0.0
        self.acc_z_avg = 0.0
        self.gyro_x_avg = 0.0
        self.gyro_y_avg = 0.0
        self.gyro_z_avg = 0.0

        #filter
        self.acc_x_filtered = 0.0
        self.acc_y_filtered = 0.0
        self.acc_z_filtered = 0.0
        self.gyro_x_filtered = 0.0
        self.gyro_y_filtered = 0.0
        self.gyro_z_filtered = 0.0
        self.alpha = LPF_A  # Filter constant, adjust as needed

        self.timer = self.create_timer(0.001, self.publish_imu_data)  # publish every 1 msec - Adjust the timer period as needed


    def low_pass_filter(self, current_value, last_filtered_value):
        return self.alpha * current_value + (1 - self.alpha) * last_filtered_value

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



    def calibrate_accelerometer(self, num_samples=SAMPLES):
        acc_x_total = 0
        acc_y_total = 0
        acc_z_total = 0

        for i in range(num_samples):
            #print(i)
            print(".", end="", flush=True)
            acc_x_total += self.read_raw_data(ACCEL_XOUT_H)
            acc_y_total += self.read_raw_data(ACCEL_YOUT_H)
            acc_z_total += self.read_raw_data(ACCEL_ZOUT_H)
            sleep(TIME_BTWEEN_SAMPLE)

        print("")
        # Average values
        self.acc_x_avg = acc_x_total / num_samples
        self.acc_y_avg = acc_y_total / num_samples
        self.acc_z_avg = acc_z_total / num_samples - 16384  # Subtract 1g (gravity)
        print("Acc offset X:"+str(self.acc_x_avg ) + " Y:"+str(self.acc_y_avg)+" Z:"+str( self.acc_z_avg))

        #return (acc_x_avg, acc_y_avg, acc_z_avg)

    def calibrate_gyroscope(self, num_samples=SAMPLES):
        gyro_x_total = 0
        gyro_y_total = 0
        gyro_z_total = 0
 

        for i in range(num_samples):
            #print(i)
            print(".", end="", flush=True)
            gyro_x_total += self.read_raw_data(GYRO_XOUT_H)
            gyro_y_total += self.read_raw_data(GYRO_YOUT_H)
            gyro_z_total += self.read_raw_data(GYRO_ZOUT_H)
            sleep(TIME_BTWEEN_SAMPLE)

        print("")
        # Average values
        self.gyro_x_avg = gyro_x_total / num_samples
        self.gyro_y_avg = gyro_y_total / num_samples
        self.gyro_z_avg = gyro_z_total / num_samples
        print("Gyro offset X:"+str(self.gyro_x_avg ) + " Y:"+str(self.gyro_y_avg)+" Z:"+str( self.gyro_z_avg))
        #return (gyro_x_avg, gyro_y_avg, gyro_z_avg)



    def handle_calibration_request(self, request, resp):
        # Code to perform calibration
        # You can add your accelerometer and gyroscope calibration logic here
        self.get_logger().info('Calibrating IMU...')
        #self.get_logger().info('Calibrating IMU with: '+str(request)+" "+str(resp))

        # Perform calibration
        self.get_logger().info('Start calibrating IMU accelerometer')
        self.calibrate_accelerometer() 
        self.get_logger().info('Start calibrating IMU gyroscope')
        self.calibrate_gyroscope()

        response = Trigger.Response()
        response.success = True
        response.message = "IMU Calibration completed successfully"
        return response





    def publish_imu_data(self):
        imu_msg = Imu()

        # Update the timestamp in the header
        now = self.get_clock().now()
        imu_msg.header.stamp.sec = now.seconds_nanoseconds()[0]
        imu_msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]
        imu_msg.header.frame_id = "mpu6050"


        # Your sensor reading and processing code here
        acc_x = self.read_raw_data(ACCEL_XOUT_H)-self.acc_x_avg
        acc_y = self.read_raw_data(ACCEL_YOUT_H)-self.acc_y_avg
        acc_z = self.read_raw_data(ACCEL_ZOUT_H)-self.acc_z_avg


        #print("Read Gyroscope raw value")
        gyro_x = self.read_raw_data(GYRO_XOUT_H)-self.gyro_x_avg
        gyro_y = self.read_raw_data(GYRO_YOUT_H)-self.gyro_y_avg
        gyro_z = self.read_raw_data(GYRO_ZOUT_H)-self.gyro_z_avg

##      #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/ACC_FACTOR
        Ay = acc_y/ACC_FACTOR
        Az = acc_z/ACC_FACTOR    
      
        Gx = gyro_x/GYRO_FACTOR
        Gy = gyro_y/GYRO_FACTOR
        Gz = gyro_z/GYRO_FACTOR

        # Apply the low-pass filter to the accelerometer data
        self.acc_x_filtered = self.low_pass_filter(Ax, self.acc_x_filtered)
        self.acc_y_filtered = self.low_pass_filter(Ay, self.acc_y_filtered)
        self.acc_z_filtered = self.low_pass_filter(Az, self.acc_z_filtered)

        # Apply the low-pass filter to the gyroscope data
        self.gyro_x_filtered = self.low_pass_filter(Gx, self.gyro_x_filtered)
        self.gyro_y_filtered = self.low_pass_filter(Gy, self.gyro_y_filtered)
        self.gyro_z_filtered = self.low_pass_filter(Gz, self.gyro_z_filtered)





        # Fill in the IMU message with the sensor data
        # Modify imu_msg.linear_acceleration and imu_msg.angular_velocity as needed
        imu_msg.linear_acceleration.x = self.acc_x_filtered
        imu_msg.linear_acceleration.y = self.acc_y_filtered
        imu_msg.linear_acceleration.z = self.acc_z_filtered
        imu_msg.angular_velocity.x = self.gyro_x_filtered
        imu_msg.angular_velocity.y = self.gyro_y_filtered
        imu_msg.angular_velocity.z = self.gyro_z_filtered
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

