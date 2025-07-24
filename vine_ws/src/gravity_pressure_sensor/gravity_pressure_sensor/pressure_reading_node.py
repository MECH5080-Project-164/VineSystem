import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from .DFRobot_MPX5700 import DFRobot_MPX5700_I2C

class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')
        self.get_logger().info('Pressure sensor node started')

        # Initialise the pressure sensor
        # I2C bus 1, sensor address 0x16
        try:
            self.pressure_sensor = DFRobot_MPX5700_I2C(1, 0x16)
            self.pressure_sensor.set_mean_sample_size(10)
            self.get_logger().info('Pressure sensor initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize pressure sensor: {str(e)}')
            self.pressure_sensor = None

        # Create a publisher for pressure readings
        self.pressure_publisher = self.create_publisher(Float32, 'pressure', 10)

        # Timer to publish pressure readings at a fixed rate
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pressure)

    def publish_pressure(self):
        msg = Float32()

        if self.pressure_sensor is not None:
            try:
                # Read pressure value in kPa
                pressure_kpa = self.pressure_sensor.get_pressure_value_kpa(1)
                msg.data = float(pressure_kpa)
                self.get_logger().info(f'Publishing pressure: {msg.data:.2f} kPa')
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
    rclpy.spin(pressure_node)
    pressure_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
