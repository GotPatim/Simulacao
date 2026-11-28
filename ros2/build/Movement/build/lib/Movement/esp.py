import math
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


# Só para testes do HWInterface
# 360 ticks por rotação
class FakeESP(Node):
    def __init__(self):
        super().__init__('fake_esp')
        self.declare_parameter('port', '/tmp/ttyESP')
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.20)
        self.declare_parameter('ticks_per_rev', 360)

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.tpr = self.get_parameter('ticks_per_rev').value

        self.ser = serial.Serial(self.get_parameter('port').value, 115200, timeout=0)
        self.rx_buf = ''
        self.pos = {'left_wheel_joint': 0.0, 'right_wheel_joint': 0.0}   # rad
        self.prev_ticks = None
        self.prev_time = None
        self.last_cmd = self.get_clock().now()

        self.gz_cmd_pub = self.create_publisher(Twist, '/gz_cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 1)
        self.create_timer(0.005, self.read_serial)
        self.create_timer(0.02, self.send_encoders)    # 50 Hz, como o ESP real
        self.create_timer(0.05, self.watchdog)

    def joint_cb(self, msg):
        for name, p in zip(msg.name, msg.position):
            self.pos[name] = p

    def read_serial(self):
        self.rx_buf += self.ser.read(self.ser.in_waiting).decode(errors='ignore')
        while '\n' in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split('\n', 1)
            p = line.strip().split(',')
            if len(p) == 3 and p[0] == 'V':
                try:
                    wl, wr = float(p[1]), float(p[2])      # rad/s
                except ValueError:
                    continue
                cmd = Twist()
                cmd.linear.x = self.r * (wl + wr) / 2.0
                cmd.angular.z = self.r * (wr - wl) / self.L
                self.gz_cmd_pub.publish(cmd)
                self.last_cmd = self.get_clock().now()

    def watchdog(self):    # o ESP real para os motores sem comando
        if (self.get_clock().now() - self.last_cmd).nanoseconds > 5e8:
            self.gz_cmd_pub.publish(Twist())

    def send_encoders(self):
        k = self.tpr / (2.0 * math.pi)
        tl = round(self.pos['left_wheel_joint'] * k)     # ticks inteiros, como um encoder
        tr = round(self.pos['right_wheel_joint'] * k)
        now = self.get_clock().now()

        wl = wr = 0.0
        if self.prev_ticks is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt > 0:
                wl = (tl - self.prev_ticks[0]) / k / dt    # rad/s a partir dos ticks
                wr = (tr - self.prev_ticks[1]) / k / dt
        self.prev_ticks, self.prev_time = (tl, tr), now

        self.ser.write(f'E,{tl},{tr},{wl:.3f},{wr:.3f}\n'.encode())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeESP())
    rclpy.shutdown()