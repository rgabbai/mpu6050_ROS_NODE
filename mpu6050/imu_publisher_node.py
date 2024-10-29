import rclpy
from sensor_msgs.msg import Imu  # IMU type msg topic 
import smbus                     #import SMBus module of I2C
from rclpy.node import Node
#from std_srvs.srv import Trigger, Trigger_Response
from std_srvs.srv import Trigger
from time import sleep  
import json
from imu_calibration_pkg.srv import CalibrateIMU


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

ACC_FACTOR   = 16384.0     #mpu6050 spec for  2g
GYRO_FACTOR  = 131.0       #mpu6050 spec for  +/-250
GYRO_TUNE    = 7.22        # for tuning 90 in reality to 90 in yaw - TBD
TEMP_A       = 340.0       # C = temp_raw/TEMP_A+TEMP_B
TEMP_B       = 36.53

# Calibration constants
SHORT_SAMPLES = 500  # Number of samples for a short calibration
LONG_SAMPLES  = 2000  # Number of samples for a long calibration
SAMPLES      = 2000  # calibration reads
TIME_BTWEEN_SAMPLE = 0.001  # 1ms
LPF_A        = 0.05   # Low pass filter alfa 
TEMP_DIFF    = 0.5    # If Temp change over 0.5C generate a warning  


#bus = smbus.SMBus(6) 	# I2C ch6 
#Device_Address = 0x68   # MPU6050 device address

class ImuPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')
        self.imu_publisher = self.create_publisher(Imu, 'mpu6050/imu/data', 10)
        #self.calibration_service = self.create_service(Trigger, 'calibrate_imu', self.handle_calibration_request)
        #self.calibration_service = self.create_service(Trigger, 'calibrate_imu', self.handle_calibration_request)
        self.calibration_service = self.create_service(
            CalibrateIMU, 'calibrate_imu', self.handle_calibration_request
        )
        self.bus = smbus.SMBus(6)   # I2C ch6
        self.Device_Address = 0x68  # MPU6050 device address
        self.MPU_Init()

        self.acc_x_avg = 0.0
        self.acc_y_avg = 0.0
        self.acc_z_avg = 0.0
        self.gyro_x_avg = 0.0
        self.gyro_y_avg = 0.0
        self.gyro_z_avg = 0.0

        if self.load_calibration_from_json():
            self.get_logger().info('MPU6050 calibration file found,loading calibration data done.')
        else:
            self.get_logger().warning('Warning MPU650 calibration file was not found.')
            self.get_logger().info('Perform the following calibration steps:')
            self.get_logger().info('1. Place mpu6050 is in correct position ')
            self.get_logger().info('2. issue service request: ros2 service call /calibrate_imu std_srvs/srv/Trigger \"\{\}\"')

 
        #filter
        self.acc_x_filtered = 0.0
        self.acc_y_filtered = 0.0
        self.acc_z_filtered = 0.0
        self.gyro_x_filtered = 0.0
        self.gyro_y_filtered = 0.0
        self.gyro_z_filtered = 0.0
        self.alpha = LPF_A  # Filter constant, adjust as needed

        self.temp = self.read_raw_data(0x41)  # 0x41 is the temperature register
        self.prev_temp = self.temp
        self.need_calb = False

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

    def save_calibration_to_json(self, filename="mpu6050_calibration.json"):
        calibration_data = {
            "acc_x_avg": self.acc_x_avg,
            "acc_y_avg": self.acc_y_avg,
            "acc_z_avg": self.acc_z_avg,
            "gyro_x_avg":self.gyro_x_avg,
            "gyro_y_avg":self.gyro_y_avg,
            "gyro_z_avg":self.gyro_z_avg
        }
        with open(filename, "w") as file:
            json.dump(calibration_data, file)


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


    def load_calibration_from_json(self, filename="mpu6050_calibration.json"):
        try:
            with open(filename, "r") as file:
                calibration_data = json.load(file)
                self.acc_x_avg = calibration_data["acc_x_avg"]
                self.acc_y_avg = calibration_data["acc_y_avg"]
                self.acc_z_avg = calibration_data["acc_z_avg"]
                self.gyro_x_avg = calibration_data["gyro_x_avg"]
                self.gyro_y_avg = calibration_data["gyro_y_avg"]
                self.gyro_z_avg = calibration_data["gyro_z_avg"]
            return True
        except FileNotFoundError:
            return False

    '''
    def handle_calibration_request(self, request, resp):
        # Code to perform calibration
        # You can add your accelerometer and gyroscope calibration logic here
        self.get_logger().info('Recived calibrating MPU650 Request...')
        #self.get_logger().info('Calibrating IMU with: '+str(request)+" "+str(resp))
        response = Trigger.Response()

        
        if self.load_calibration_from_json():
            self.get_logger().info('MPU650 calibration file was found,loading calibration data done.')
            self.get_logger().warning('If you desire to recalibrate delet the file: mpu6050_calibration.json first.')
            response.success = True
            response.message = "IMU Calibration loaded successfully"
            return response
        
        else:
            # Perform calibration
            self.get_logger().info('Start calibrating IMU accelerometer')
            self.calibrate_accelerometer() 
            self.get_logger().info('Start calibrating IMU gyroscope')
            self.calibrate_gyroscope()
            self.save_calibration_to_json()
            response.success = True
            response.message = "IMU Calibration process completed successfully and saved to file:mpu650_calibration.json "
            return response
    '''
    def handle_calibration_request(self, request, response):
        self.get_logger().info(f'Received calibration request with mode: {request.mode}')

        if request.mode == "load":
            if self.load_calibration_from_json():
                self.get_logger().info('Calibration file loaded successfully.')
                response.success = True
                response.message = "Loaded calibration data from file."
            else:
                self.get_logger().warning('No calibration file found. Please perform calibration.')
                response.success = False
                response.message = "Calibration file not found. Please perform calibration."
            return response

        elif request.mode == "short":
            self.get_logger().info('Performing short calibration of IMU.')
            self.calibrate_accelerometer(SHORT_SAMPLES)
            self.calibrate_gyroscope(SHORT_SAMPLES)
            self.save_calibration_to_json()
            response.success = True
            response.message = "Short calibration completed and saved."

        elif request.mode == "long":
            self.get_logger().info('Performing long calibration of IMU.')
            self.calibrate_accelerometer(LONG_SAMPLES)
            self.calibrate_gyroscope(LONG_SAMPLES)
            self.save_calibration_to_json()
            response.success = True
            response.message = "Long calibration completed and saved."
        elif request.mode == "temp_check":
            self.get_logger().info('check if need to calibrate due temp chage.')
            if self.need_calb:
                self.calibrate_accelerometer(SHORT_SAMPLES)
                self.calibrate_gyroscope(SHORT_SAMPLES)
                self.save_calibration_to_json()
                response.success = True
                response.message = "Short calibration completed and saved due temp change."
                self.need_calb = False
            else: 
                response.success = True
                response.message = "No need to recalibrate - No temp change."
        else:
            self.get_logger().warning(f"Unknown calibration mode: {request.mode}")
            response.success = False
            response.message = f"Unknown mode: {request.mode}"

        return response



    def read_temperature(self):
        temp_raw = self.read_raw_data(0x41)  # 0x41 is the temperature register
        temperature = temp_raw / TEMP_A + TEMP_B
        return temperature



    def publish_imu_data(self):
        imu_msg = Imu()

        # Update the timestamp in the header
        now = self.get_clock().now()
        imu_msg.header.stamp.sec = now.seconds_nanoseconds()[0]
        imu_msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]
        imu_msg.header.frame_id = "mpu6050"

        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.01] * 9  # Adjust as necessary
        imu_msg.linear_acceleration_covariance = [0.1] * 9  # Adjust as necessary


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
      
        Gx = gyro_x/GYRO_FACTOR/GYRO_TUNE
        Gy = gyro_y/GYRO_FACTOR/GYRO_TUNE
        Gz = gyro_z/GYRO_FACTOR/GYRO_TUNE

        # Apply the low-pass filter to the accelerometer data
        self.acc_x_filtered = self.low_pass_filter(Ax, self.acc_x_filtered)
        self.acc_y_filtered = self.low_pass_filter(Ay, self.acc_y_filtered)
        self.acc_z_filtered = self.low_pass_filter(Az, self.acc_z_filtered)

        # Apply the low-pass filter to the gyroscope data
        self.gyro_x_filtered = self.low_pass_filter(Gx, self.gyro_x_filtered)
        self.gyro_y_filtered = self.low_pass_filter(Gy, self.gyro_y_filtered)
        self.gyro_z_filtered = self.low_pass_filter(Gz, self.gyro_z_filtered)

        # Read and log temperature - might need to do compensations due bias changes
        self.temp = self.read_temperature()
        if abs(self.temp-self.prev_temp) > TEMP_DIFF:
            self.get_logger().info(f"Warning MPU6050 Temperature change: {self.temp:.2f}°C")
            self.need_calb = True
        self.prev_temp = self.temp



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

