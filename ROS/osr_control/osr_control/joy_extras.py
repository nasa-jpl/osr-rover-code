import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup


class JoyButtonSubscriber(Node):
    """For custom functionality that the joy to twist publisher node can't handle"""
    def __init__(self):
        super().__init__('joy_button_subscriber')
        self.log = self.get_logger()
        self.declare_parameter('duty_button_index', 0)  # Default button index is 0
        self.duty_button_index = self.get_parameter('duty_button_index').value
        self.last_duty_mode_value = False
        self.last_received_time = None        
        self.cb_group = ReentrantCallbackGroup()
        self.roboclaw_node_set_param_client = self.create_client(SetParameters, '/roboclaw_wrapper/set_parameters', 
                                                                 callback_group=self.cb_group)
        while not self.roboclaw_node_set_param_client.wait_for_service(timeout_sec=5.0):
            self.log.info('Service /roboclaw_wrapper/set_parameters not available, waiting again...', skip_first=True)
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.log.info("Initialized JoyButtonSubscriber.")

    async def joy_callback(self, msg):
        if len(msg.buttons) <= self.duty_button_index:
            error_msg = f"button index {self.duty_button_index} exceeds the number of buttons available ({len(msg.buttons)})"
            self.log.error(error_msg)
            raise AttributeError(error_msg)

        button_value = msg.buttons[self.duty_button_index]
        current_time = self.get_clock().now()
        # only flip the mode if we haven't received a button press in over a second
        if button_value and (self.last_received_time is None or (current_time - self.last_received_time).nanoseconds > 1e9):
            # change the duty mode to the opposite of what it was
            param = Parameter(name="duty_mode", value=not self.last_duty_mode_value).to_parameter_msg()
            self.log.info(f"Turning duty mode {'off' if self.last_duty_mode_value else 'on'}")
            self.last_duty_mode_value = not self.last_duty_mode_value
            await self.set_parameter_value_on_other_node(self.roboclaw_node_set_param_client, param)
            self.last_received_time = current_time

    async def set_parameter_value_on_other_node(self, node_set_param_client, param):
        request = SetParameters.Request()
        request.parameters.append(param)
        future = node_set_param_client.call_async(request)
        try:
            result = await future
        except Exception as e:
            self.log.warning(f"Failed to call service with parameter {param.name}: {e}")
        else:
            res = result.results[0]
            if not res.successful:
                self.log.warning(f"Service call with parameter {param.name} unsuccessful: {res.reason}")

def main(args=None):
    rclpy.init(args=args)
    joy_button_subscriber = JoyButtonSubscriber()
    rclpy.spin(joy_button_subscriber)
    joy_button_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
