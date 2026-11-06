import rclpy
import math
from tf2_ros import TransformBroadcaster
import transforms3d
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from Movement.robot_constants import *
import serial


SERIAL_PORT = '/tmp/ttyHW'
BAUD_RATE = 115200

class HWInterface(Node): #Refazer 
    # Vai receber 2 velocidades da esp(direita e esquerda)
    # Vai converter pro que a odometria entende ( linear e angular)
    # vai enviar as velocidades calculadas no walk pra esp em formato de direita e esquerda)

    def __init__(self):
        super().__init__('HWInterface')

        self.connection = None
        self.connected = False
        
        try:
            self.setupVariables()
            self.connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.waitFor(2)  # Aguardar inicialização por 2 segundos
            self.connected = True
            print("Interface de Hardware conectada ao ESP32.")

            self.cmd_sub =  self.create_subscription(Twist,"/cmd_vel", self.sendSpeed, 10)
            self.odom_pub =  self.create_publisher(Odometry,"/odom", 50)

            self.create_timer(0.02, self.calcOdom) # posta a odometria pro cmd a 50hz
            self.get_logger().info(f"ABAXACI")


        except serial.SerialException as e:
            print(f"Erro Crítico: Não foi possível conectar ao hardware. {e}")
            self.connected = False
            return


    def waitFor(self, seconds):
        now = self.get_clock().now()
        if self.wait_start is None:
            self.wait_start = now

        while ((now - self.wait_start).nanoseconds / 1e9 < seconds):
            now = self.get_clock().now()
            if (now - self.wait_start).nanoseconds / 1e9 >= seconds:
                self.wait_start = None      # reset so it works again next time
        return True

    def setupVariables(self):
        #   Control
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.wait_start = None
        self.buffer = ""
        
        #   Current Position
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        
        #   Speed
        self.current_linear_speed_x = 0
        self.current_linear_speed_y = 0
        self.current_angular_speed_z = 0 # rad/s

    def normalize_angle(self, angle): # Limits the angle between -180 and 180
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def receiveData(self):

        self.get_logger().info(f"BATATAAA")
        if (self.connection and self.connection.in_waiting > 0):
            data = self.connection.read(self.connection.in_waiting).decode("utf-8")
            self.get_logger().info(f"Read data: {data}")

            self.buffer += data # ---> b'E,120,118,1.234,1.100\nE,125,12'

            # Utiliza somente a ultima linha pra simplificar o processo 
            lines = self.buffer.split("\n")
            line = lines[-2]
            self.buffer = lines[-1] # guarda a linha incompleta pra que proximo ciclo não fique desincronizado
            line = line.strip() # fica só os numeros eas virgulas ------> E,120,118,1.234,1.100
            info = line.split(",")
            
            vel_left = info[3]
            vel_right = info[4]

            return vel_left, vel_right



    def calcOdom(self): 
        # Recebe a velocidade dos motores e calcula: 
        # velocidade_linear_x ( velocidade do centro do robo na direção x)
        # velocidade_linear_y (velocidade do centro no eixo y)
        # velocidade angular ( velocidade com a qual o robo está virando)  
        # posicao_x (posição atual dps de andar um pouco em x )  
        # posicao_y (posição atual dps de andar um pouco em y )
        # Angulo que o robo está virado em relação aos eixos x e y

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.get_logger().info(f"ABAXACI")

        # Recebe as info da ESP..
        vel_left, vel_right = self.receiveData() # Adicionar mais segurança caso a fila estaj vazia 
        #...
        # Trata os dados pro formato q o cmd_vel usa

        self.current_angular_speed_z = (WHEEL_RADIUS_RIGHT * vel_right - WHEEL_RADIUS_LEFT * vel_left) / WHEEL_SEPARATION 
        self.current_yaw += self.current_angular_speed_z * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)

        self.current_linear_speed_x = (WHEEL_RADIUS_RIGHT * vel_right + WHEEL_RADIUS_LEFT * vel_left) / 2 

        self.current_x += self.current_linear_speed_x * math.cos(self.current_yaw) * dt
        self.current_y += self.current_linear_speed_x * math.sin(self.current_yaw) * dt

        self.last_time = current_time

        # Preparing to publish

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.current_yaw / 2.0)
        q.w = math.cos(self.current_yaw / 2.0)

        msg = Odometry()
        msg.header.stamp           =  self.get_clock().now.to_msg()
        msg.header.frame_id       =  "odom"

        msg.pose.pose.position.x  =  self.current_x
        msg.pose.pose.position.y  =  self.current_y
        msg.pose.pose.position.z  =  0
        msg.pose.pose.orientation =  q

        msg.child_frame_id        =  "base_link"
        
        msg.twist.twist.linear.x  =  self.current_linear_speed_x
        msg.twist.twist.linear.y  =  0
        msg.twist.twist.angular.z =  self.current_angular_speed_z
        
        self.odom_pub.publish(msg)
    
    def sendSpeed(self, msg):
        if not self.connected or not self.connection:
            print("Hardware não conectado, ignorando comando.")
            return

        vel_left = msg.linear.x - msg.angular.z * WHEEL_SEPARATION/2
        vel_right = msg.linear.x + msg.angular.z * WHEEL_SEPARATION/2
        
        wl = vel_left / WHEEL_RADIUS_LEFT
        wr = vel_right / WHEEL_RADIUS_RIGHT

        command = f"V,{wl:.3f},{wr:.3f}\n"
        try:
            self.connection.write(command.encode('utf-8'))
            print(f"Comando enviado: {command.strip()}")

        except serial.SerialException as e:
            print(f"Erro ao enviar comando: {e}")
            self.connected = False

    
    def close(self):
        self.is_running = False
        self.connected = False
        if self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1)
        if self.connection:
            self.connection.close()
        print("Interface de Hardware desconectada.")
        

def main(args=None):
    rclpy.init(args=args)
    node = HWInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()