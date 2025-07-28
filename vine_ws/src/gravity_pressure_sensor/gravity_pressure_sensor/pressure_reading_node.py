import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .DFRobot_MPX5700_optimized import DFRobot_MPX5700_I2C_Optimized

class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')

        # Parameters for the pressure sensor
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x16)
        self.declare_parameter('publish_rate_hz', 50.0)  # Increased default to 50Hz
        self.declare_parameter('mean_sample_size', 10)

        # Get parameters
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().value
        self.mean_sample_size = self.get_parameter('mean_sample_size').get_parameter_value().value

        self.get_logger().info('Pressure sensor node started')
        self.get_logger().info(
            f'Using I2C bus {i2c_bus} at address 0x{i2c_address:02X} '
            f'with {self.publish_rate_hz}Hz publish rate.'
        )

        # Initialise the pressure sensor
        # I2C bus 1, sensor address 0x16
        try:
            self.pressure_sensor = DFRobot_MPX5700_I2C_Optimized(i2c_bus, i2c_address)
            self.pressure_sensor.set_mean_sample_size(self.mean_sample_size)
            self.get_logger().info('Pressure sensor initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize pressure sensor: {str(e)}')
            self.pressure_sensor = None

        # Create a publisher for pressure readings
        self.pressure_publisher = self.create_publisher(Float32, 'pressure', 10)

        # Timer to publish pressure readings at a fixed rate
        timer_period = 1.0 / self.publish_rate_hz  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pressure)

    def publish_pressure(self):
        msg = Float32()

        if self.pressure_sensor is not None:
            try:
                # Use the faster pressure reading method
                pressure_kpa = self.pressure_sensor.get_pressure_value_kpa_fast()
                if pressure_kpa != -1.0:
                    msg.data = float(pressure_kpa)
                    # Reduce logging frequency to avoid spam at 50Hz
                    if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Log ~once per second
                        self.get_logger().info(f'Publishing pressure: {msg.data:.2f} kPa')
                else:
                    # Fallback to original method if fast method fails
                    pressure_kpa = self.pressure_sensor.get_pressure_value_kpa(1)
                    msg.data = float(pressure_kpa)
            except Exception as e:
                self.get_logger().error(f'Error reading pressure sensor: {str(e)}')
                msg.data = -1.0  # Error value
        else:
            msg.data = -1.0  # Sensor not available
            self.get_logger().warn('Pressure sensor not available, publishing error value')

        self.pressure_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pressure_node = PressureSensorNode()
    try:
        rclpy.spin(pressure_node)
    except KeyboardInterrupt:
        pass
    finally:
        pressure_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
