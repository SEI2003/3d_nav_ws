from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Twist
import time

class KobukiForward(EventState):
    """
    FlexBE state to make Kobuki move forward.
    """

    def __init__(self, linear_speed):
        """
        Initialize the KobukiForward state.
        linear_speed: Linear speed of Kobuki (m/s).
        """
        super(KobukiForward, self).__init__(outcomes=['success', 'failed'])

        self._linear_speed = linear_speed
        #self._cmd_vel_pub = rospy.Publisher('commands/velocity', Twist, queue_size=1)

        #self.node = rclpy.create_node('kobuki_forward')
        ProxyPublisher.initialize(KobukiForward._node)
        self.pub = ProxyPublisher()
        self.pub.createPublisher('commands/velocity', Twist)


    def execute(self, userdata):
        """
        Execute the state. Publishes Twist command to move Kobuki forward.
        """
        twist_cmd = Twist()
        twist_cmd.linear.x = self._linear_speed
        twist_cmd.angular.z = 0.0

        self.pub.publish('commands/velocity',twist_cmd)
        print(F"{twist_cmd}")
        time.sleep(0.5)
        # Assuming successful execution
        return 'success'

    def on_exit(self, userdata):
        """
        Cleanup behavior.
        """
        # Stop Kobuki when exiting state
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0

        self.pub.publish('commands/velocity',twist_cmd)

        Logger.loginfo("Stopping Kobuki")