import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from std_msgs.msg import Int16MultiArray;

class Gantry:
    def __init__(self):
        self.initialized = True
        self.gantry_send_msg = Int16MultiArray()
        self.gantry_mode_msg = Int16()
        self.gantry_cmd_pub = rospy.Publisher("gantry_cmd_send", Int16MultiArray, queue_size=10)
        self.gantry_mode_pub = rospy.Publisher("gantry_cmd_hack_send", Int16, queue_size=10)

    def send_pos_cmd(self, pos_cmd):
        self.pos_cmd = pos_cmd
        # self.gantry_send_msg = Point(*pos_cmd)
        # self.gantry_send_msg = Int16MultiArray(pos_cmd)
        self.gantry_send_msg.data = pos_cmd;
        self.gantry_cmd_pub.publish(self.gantry_send_msg)

    def send_state(self, state):
        self.gantry_mode_msg.data = state
        self.gantry_mode_pub.publish(self.gantry_mode_msg)
