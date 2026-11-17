#!/usr/bin/env python3
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# Registres MPU6050
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B   # début du bloc : accel(6) + temp(2) + gyro(6) = 14 octets
DEVICE_ADDRESS = 0x68

# Plages de mesure et facteurs de conversion
GYRO_RANGE_250  = 0x00            # ±250 °/s  -> 131 LSB/(°/s)
ACCEL_RANGE_2G  = 0x00            # ±2 g      -> 16384 LSB/g
GYRO_SCALE  = 131.0 * 57.29578    # LSB -> rad/s  (≈ 7505.75)
ACCEL_SCALE = 16384.0 / 9.80665   # LSB -> m/s²   (≈ 1670.70)

# Variances (diagonale des covariances) : estimations pour un MPU6050
GYRO_VARIANCE  = 1e-4   # (rad/s)²
ACCEL_VARIANCE = 1e-2   # (m/s²)²


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")

        # Paramètre : numéro du bus I2C (Jetson Orin Nano, broches 3/5 -> bus 7)
        self.declare_parameter("i2c_bus", 7)
        self.i2c_bus_ = self.get_parameter("i2c_bus").value

        # Paramètre : nombre d'échantillons de calibration (200 = 2 s à 100 Hz).
        # 0 désactive la calibration.
        self.declare_parameter("calibration_samples", 200)
        self.calibration_samples_ = self.get_parameter("calibration_samples").value

        # Biais du gyroscope, mesuré au démarrage pendant que le robot est immobile
        self.gyro_bias_ = [0.0, 0.0, 0.0]
        self.calibration_sum_ = [0.0, 0.0, 0.0]
        self.calibration_count_ = 0
        self.is_calibrated_ = (self.calibration_samples_ <= 0)

        # Interface I2C
        self.bus_ = None
        self.is_connected_ = False
        self.init_i2c()

        # Interface ROS 2
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"

        # Pas d'orientation fournie : covariance[0] = -1 (convention REP-145)
        self.imu_msg_.orientation_covariance[0] = -1.0
        self.imu_msg_.angular_velocity_covariance = [GYRO_VARIANCE, 0.0, 0.0,
                                                     0.0, GYRO_VARIANCE, 0.0,
                                                     0.0, 0.0, GYRO_VARIANCE]
        self.imu_msg_.linear_acceleration_covariance = [ACCEL_VARIANCE, 0.0, 0.0,
                                                        0.0, ACCEL_VARIANCE, 0.0,
                                                        0.0, 0.0, ACCEL_VARIANCE]

        self.period_ = 0.01   # s -> 100 Hz
        self.timer_ = self.create_timer(self.period_, self.timerCallback)

        if not self.is_calibrated_:
            self.get_logger().info(
                "Calibration du gyroscope sur %d échantillons : NE PAS BOUGER LE ROBOT."
                % self.calibration_samples_)

    def timerCallback(self):
        # Capteur absent : on retente la connexion et on sort si elle échoue
        if not self.is_connected_:
            self.init_i2c()
            if not self.is_connected_:
                return

        try:
            # Une seule transaction I2C pour les 14 octets (valeurs cohérentes entre elles)
            data = self.bus_.read_i2c_block_data(DEVICE_ADDRESS, ACCEL_XOUT_H, 14)
        except OSError:
            self.is_connected_ = False
            self.get_logger().warn("MPU6050 lost, trying to reconnect...")
            return

        acc_x  = self.to_signed(data[0],  data[1])
        acc_y  = self.to_signed(data[2],  data[3])
        acc_z  = self.to_signed(data[4],  data[5])
        # data[6], data[7] : température (non utilisée)
        gyro_x = self.to_signed(data[8],  data[9])
        gyro_y = self.to_signed(data[10], data[11])
        gyro_z = self.to_signed(data[12], data[13])

        gyro = [gyro_x / GYRO_SCALE, gyro_y / GYRO_SCALE, gyro_z / GYRO_SCALE]   # rad/s

        # Calibration : au repos, le gyroscope devrait lire 0. Ce qu'il lit est son biais.
        # On le moyenne sur quelques secondes, puis on le soustrait de toutes les mesures.
        if not self.is_calibrated_:
            self.collectCalibration(gyro)
            return

        self.imu_msg_.linear_acceleration.x = acc_x / ACCEL_SCALE
        self.imu_msg_.linear_acceleration.y = acc_y / ACCEL_SCALE
        self.imu_msg_.linear_acceleration.z = acc_z / ACCEL_SCALE
        self.imu_msg_.angular_velocity.x = gyro[0] - self.gyro_bias_[0]
        self.imu_msg_.angular_velocity.y = gyro[1] - self.gyro_bias_[1]
        self.imu_msg_.angular_velocity.z = gyro[2] - self.gyro_bias_[2]

        self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub_.publish(self.imu_msg_)

    def collectCalibration(self, gyro):
        """Accumule les mesures au repos et calcule le biais une fois le compte atteint."""
        for axis in range(3):
            self.calibration_sum_[axis] += gyro[axis]
        self.calibration_count_ += 1

        if self.calibration_count_ < self.calibration_samples_:
            return

        self.gyro_bias_ = [total / self.calibration_count_ for total in self.calibration_sum_]
        self.is_calibrated_ = True
        self.get_logger().info(
            "Biais du gyroscope : x=%.4f  y=%.4f  z=%.4f rad/s" % tuple(self.gyro_bias_))
        if max(abs(bias) for bias in self.gyro_bias_) > 0.5:
            self.get_logger().warn(
                "Biais inhabituellement élevé : le robot a-t-il bougé pendant la calibration ?")

    def init_i2c(self):
        try:
            if self.bus_ is None:
                self.bus_ = smbus.SMBus(self.i2c_bus_)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)      # réveil, horloge gyro X
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, GYRO_RANGE_250)
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, ACCEL_RANGE_2G)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
            self.get_logger().info("MPU6050 connected on /dev/i2c-%d" % self.i2c_bus_)
        except OSError as e:
            self.is_connected_ = False
            self.get_logger().warn(
                "MPU6050 not found on /dev/i2c-%d (%s)" % (self.i2c_bus_, e),
                throttle_duration_sec=5.0)

    @staticmethod
    def to_signed(high, low):
        # Valeur 16 bits en complément à deux
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
