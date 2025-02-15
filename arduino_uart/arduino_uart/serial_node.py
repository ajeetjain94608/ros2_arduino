import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        # Open Serial Connection with Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # ROS2 Publisher for MPU6050 sensor data
        self.sensor_publisher = self.create_publisher(String, 'mpu_data', 10)

        # ROS2 Subscriber for LED control
        self.led_subscriber = self.create_subscription(String, 'led_control', self.led_callback, 10)

        # Timer to read sensor data from Arduino
        self.timer = self.create_timer(1.0, self.read_sensor_data)

    def led_callback(self, msg):
        """Send LED control commands to Arduino over UART."""
        self.serial_port.write((msg.data + "\n").encode())
        self.get_logger().info(f"Sent LED Command: {msg.data}")

    def read_sensor_data(self):
        """Read sensor data from Arduino and publish to ROS2."""
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode().strip()
            if data:
                self.get_logger().info(f"Received MPU6050 Data: {data}")
                msg = String()
                msg.data = data
                self.sensor_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
