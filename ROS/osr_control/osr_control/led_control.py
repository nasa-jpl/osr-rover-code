import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import UInt8MultiArray
from adafruit_servokit import ServoKit


class LEDControlNode(Node):
    """
    Node to control the intensity of two LEDs connected to channels 4 and 5 of the PCA9685.
    The intensity is controlled by publishing two percentages (0-100) to the topic '/led_intensity'.
    """

    def __init__(self):
        super().__init__('led_control_node')
        self.log = self.get_logger()
        self.log.info("Initializing LED Control Node")

        # Initialize I2C and PCA9685
        self.kit = ServoKit(channels=16)

        # LED channels
        self.led_channels = [4, 5]

        # Create subscription to control LED intensity
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/led_intensity',
            self.led_intensity_callback,
            qos_profile
        )

    def led_intensity_callback(self, msg: UInt8MultiArray):
        """
        Callback to set the intensity of the LEDs based on the received message.
        The message should contain two float values (0-100) representing the intensity percentages.
        """
        if len(msg.data) != 2:
            self.log.error("Received message does not contain exactly two values. Ignoring.")
            return

        for i, intensity in enumerate(msg.data):
            if not intensity <= 100:
                self.log.error(f"Invalid intensity value {intensity}. Must be between 0 and 100.")
                return

            # Convert percentage to an angle
            angle = int((intensity / 100.0) * 180)
            self.kit.servo[self.led_channels[i]].angle = angle
            self.log.debug(f"Set LED on channel {self.led_channels[i]} to {intensity}% intensity.")


def main(args=None):
    rclpy.init(args=args)
    node = LEDControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()