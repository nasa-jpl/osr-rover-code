import rclpy
from rclpy.node import Node

# project libraries
from adafruit_servokit import ServoKit


# message imports
from osr_interfaces.msg import CommandCorner, Status


class ServoWrapper(Node):
    """Interface between the PCA9685 controlling the servos and the higher level rover code"""
    def __init__(self):
        super().__init__("servo_wrapper")
        self.log = self.get_logger()
        self.log.set_level(10)
        self.log.info("Initializing corner servo controllers")
        self.kit = None

        self.connect_pca9685()        
        
        self.corner_cmd_sub = self.create_subscription(CommandCorner, "/cmd_corner", self.corner_cmd_cb, 1)

    def connect_pca9685(self):
        self.kit = ServoKit(channels=16)


    def corner_cmd_cb(self, cmd: CommandCorner):
        self.log.debug(f"Received corner command message: {cmd}")
        if not self.kit:
            self.log.error("ServoKit not instantiated yet, dropping cmd", throttle_sec=5)
            return

        self.kit.servo[0].angle = max(min(cmd.left_back_pos * 180/3.14, 100), 0)
        self.kit.servo[1].angle = max(min(cmd.left_front_pos * 180/3.14, 100), 0)
        self.kit.servo[2].angle = max(min(cmd.right_front_pos * 180/3.14, 100), 0)
        self.kit.servo[3].angle = max(min(cmd.right_back_pos * 180/3.14, 100), 0)




def main(args=None):
    rclpy.init(args=args)

    wrapper = ServoWrapper()

    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
