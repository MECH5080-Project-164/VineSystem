import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import signal
import sys
from .DFRobot_MPX5700 import DFRobot_MPX5700_I2C

class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')

        # Parameters for the pressure sensor
        # self.declare_parameter('i2c_bus', 1)
        # self.declare_parameter('i2c_address', 0x16)
        # self.declare_parameter('publish_rate_hz', 1.0)
        # self.declare_parameter('mean_sample_size', 10)

        # # Get parameters
        # i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().value
        # i2c_address = self.get_parameter('i2c_address').get_parameter_value().value
        # self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().value
        # self.mean_sample_size = self.get_parameter('mean_sample_size').get_parameter_value().value

        i2c_bus = 1
        i2c_address = 0x16
        self.publish_rate_hz = 50
        self.mean_sample_size = 3


        self.get_logger().info('Pressure sensor node started')
        self.get_logger().info(
            f'Using I2C bus {i2c_bus} at address 0x{i2c_address:02X} '
            f'with {self.publish_rate_hz}Hz publish rate.'
        )

        # Initialise the pressure sensor
        # I2C bus 1, sensor address 0x16
        try:
            self.pressure_sensor = DFRobot_MPX5700_I2C(i2c_bus, i2c_address)
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

        # Counter for throttled logging
        self.log_counter = 0
        self.log_every_n = 10  # Log every 10th reading (at 50Hz = 5Hz log rate)

        # Shutdown flag for graceful exit
        self.shutdown_requested = False

    def destroy_node(self):
        """Graceful shutdown of the pressure sensor node"""
        self.get_logger().info('Shutting down pressure sensor node...')
        self.shutdown_requested = True

        # Stop the timer
        if hasattr(self, 'timer'):
            self.timer.cancel()
            self.get_logger().info('Timer stopped')

        # Clean up sensor resources
        if hasattr(self, 'pressure_sensor') and self.pressure_sensor is not None:
            try:
                # If the sensor has a cleanup method, call it
                # self.pressure_sensor.cleanup()  # Uncomment if available
                self.get_logger().info('Pressure sensor resources cleaned up')
            except Exception as e:
                self.get_logger().warn(f'Error during sensor cleanup: {str(e)}')

        self.get_logger().info('Pressure sensor node shutdown complete')
        super().destroy_node()

    def publish_pressure(self):
        # Check if shutdown was requested
        if self.shutdown_requested:
            return

        msg = Float32()

        if self.pressure_sensor is not None:
            try:
                # Read pressure value in kPa
                pressure_kpa = self.pressure_sensor.get_pressure_value_kpa(1)
                msg.data = float(pressure_kpa)

                # Throttle logging to avoid console spam at 50Hz
                self.log_counter += 1
                if self.log_counter >= self.log_every_n:
                    self.get_logger().info(f'Publishing pressure: {msg.data:.2f} kPa (Rate: {self.publish_rate_hz}Hz)')
                    self.log_counter = 0

            except Exception as e:
                self.get_logger().error(f'Error reading pressure sensor: {str(e)}')
                msg.data = -1.0  # Error value
        else:
            msg.data = -1.0  # Sensor not available
            self.get_logger().warn('Pressure sensor not available, publishing error value')

        self.pressure_publisher.publish(msg)


def signal_handler(signum, frame):
    """Handle SIGINT (Ctrl+C) gracefully"""
    print('\nReceived shutdown signal (Ctrl+C)')
    print('Initiating graceful shutdown...')
    # The main loop will handle the actual shutdown


def main(args=None):
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)
    pressure_node = PressureSensorNode()

    pressure_node.get_logger().info('Pressure sensor node started - Press Ctrl+C to shutdown gracefully')

    try:
        rclpy.spin(pressure_node)
    except KeyboardInterrupt:
        # This catches the KeyboardInterrupt after signal handler
        pressure_node.get_logger().info('Processing shutdown request...')
    except Exception as e:
        pressure_node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        # Ensure graceful cleanup
        pressure_node.get_logger().info('Cleaning up resources...')

        try:
            pressure_node.destroy_node()
        except Exception as e:
            print(f'Error during node cleanup: {str(e)}')

        try:
            rclpy.shutdown()
            pressure_node.get_logger().info('Goodbye!')
        except Exception as e:
            print(f'Error during ROS shutdown: {str(e)}')

        print('Pressure sensor node shutdown complete')

if __name__ == '__main__':
    main()
