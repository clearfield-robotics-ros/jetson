import rospy
from std_msgs.msg import Int16, Int16MultiArray


class Gantry:
    def __init__(self):
        self.handshake = False
        self.command_arrived = True
        self.pos_cmd_erached = False
        self.carriage_pos = 0.0
        self.initialized = False
        self.mode = 0  # idle
        self.pos_cmd = 0
        self.gantry_send_msg = Int16MultiArray()
        self.gantry_mode_msg = Int16()
        self.gantry_cmd_pub = rospy.Publisher("gantry_cmd_send", Int16MultiArray, queue_size=10)
        self.gantry_mode_pub = rospy.Publisher("gantry_cmd_hack_send", Int16, queue_size=10)
        self.gantry_status_sub = rospy.Subscriber("gantryStat", Int16MultiArray, self.status_update)

    def status_update(self, data):
        self.mode = data.data[0]
        self.initialized = bool(data.data[1])
        pos_cmd_reached = bool(data.data[2])
        if not pos_cmd_reached:
            self.handshake = True
        self.carriage_pos = data.data[3] / 1000.0

    def not_waiting(self):
        if self.handshake:
            self.command_arrived = True
        if self.command_arrived:
            return True
        return False

    def send_pos_cmd(self, pos_cmd):
        self.handshake = False
        self.command_arrived = False

        self.pos_cmd = pos_cmd

        package = [1, int(pos_cmd*1000)]
        self.gantry_send_msg = Int16MultiArray()
        self.gantry_send_msg.data = package
        self.gantry_cmd_pub.publish(self.gantry_send_msg)

    def send_idle_cmd(self):
        package = [0, 0]
        self.gantry_send_msg = Int16MultiArray(package)
        self.gantry_cmd_pub.publish(self.gantry_send_msg)

    def send_state(self, state):
        self.gantry_mode_msg.data = state
        self.gantry_mode_pub.publish(self.gantry_mode_msg)
