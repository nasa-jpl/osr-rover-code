import rclpy
from rclpy.node import Node
import logging
from roboclaw import Roboclaw
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from osr_interfaces.msg import CommandCorner
from osr_interfaces.msg import CommandDrive
from osr_interfaces.msg import Status


class RoboclawWrapper(Node):
    def __init__(self):
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        )
        self.logger = logging.getLogger("RoboclawWrapper")
        super().__init__("roboclaw_wrapper")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("serial_ports", ["/dev/ttyUSB0"]),
                ("baud_rate", 115200),
                ("addresses", [128]),
                ("max_speed", 3.0),
                ("ticks_per_rev", 5440.0),
                ("wheel_radius", 0.075),
                ("wheel_base", 0.4826),
                ("encoder_ppr", 1000),
                ("debug_mode", False),
            ],
        )

        self.serial_ports = self.get_parameter("serial_ports").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.addresses = self.get_parameter("addresses").value
        self.max_speed = self.get_parameter("max_speed").value

        self.logger.info(
            f"Initializing with serial_ports: {self.serial_ports}, addresses: {self.addresses}"
        )

        self.rc = None
        self.establish_roboclaw_connections()

        self.drive_pub = self.create_publisher(CommandDrive, "drive_cmd", 10)
        self.corner_pub = self.create_publisher(CommandCorner, "corner_cmd", 10)
        self.status_pub = self.create_publisher(Status, "status", 10)
        self.twist_sub = self.create_subscription(
            Twist, "cmd_vel", self.twist_callback, 10
        )

        self.logger.info("RoboclawWrapper initialized")

    def establish_roboclaw_connections(self):
        self.logger.info("Connecting to Roboclaws")
        for port in self.serial_ports:
            try:
                self.logger.debug(f"Attempting to connect to {port}")
                self.rc = Roboclaw(port, self.baud_rate)
                if self.rc.Open():
                    for address in self.addresses:
                        try:
                            version = self.rc.ReadVersion(address)
                            if version[0]:
                                self.logger.info(
                                    f"Connected to Roboclaw at address {address}"
                                )
                                self.logger.info(
                                    f'Roboclaw version: {str(version[0].decode().replace(chr(0), ""))}'
                                )
                            else:
                                self.logger.error(
                                    f"Unable to get roboclaw version for address {address}"
                                )
                        except Exception as e:
                            self.logger.error(
                                f'Unable to connect to roboclaw at "{address}": {str(e)}'
                            )
                    return
            except Exception as e:
                self.logger.error(f"Error opening serial port {port}: {str(e)}")

        self.logger.error("Unable to open any serial ports")
        raise Exception("Unable to open any serial ports")

    def twist_callback(self, msg):
        self.logger.debug(f"Received twist command: {msg}")
        drive_msg = CommandDrive()
        corner_msg = CommandCorner()
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Update these lines to use the correct attribute names
        drive_msg.left_front_vel = linear_speed - angular_speed
        drive_msg.left_middle_vel = linear_speed - angular_speed
        drive_msg.left_back_vel = linear_speed - angular_speed
        drive_msg.right_front_vel = linear_speed + angular_speed
        drive_msg.right_middle_vel = linear_speed + angular_speed
        drive_msg.right_back_vel = linear_speed + angular_speed

        self.drive_pub.publish(drive_msg)
        self.corner_pub.publish(corner_msg)

        # Create and publish a Status message
        status_msg = Status()
        status_msg.battery = 0.0  # Set this to the actual battery voltage if available
        status_msg.error_status = ['0x00'] * 5  # Set to actual error statuses if available
        status_msg.temp = [0.0] * 5  # Set to actual temperatures if available
        status_msg.current = [0.0] * 10  # Set to actual current readings if available

        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    roboclaw_wrapper = RoboclawWrapper()
    rclpy.spin(roboclaw_wrapper)
    roboclaw_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
