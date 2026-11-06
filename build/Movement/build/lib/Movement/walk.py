import rclpy
import math
from tf2_ros import TransformBroadcaster
import transforms3d
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from Movement.robot_constants import *
import serial


SERIAL_PORT = '/dev/ttyUSB0'
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
            self.connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.waitFor(2)  # Aguardar inicialização por 2 segundos
            self.connected = True
            print("Interface de Hardware conectada ao ESP32.")

            self.cmd_sub =  self.create_subscription(Twist,"/cmd_vel", self.sendSpeed, 10)
            self.odom_pub =  self.create_publisher(Odometry,"/odom", 50)

            self.setupVariables()
            self.create_timer(0.02, self.calcOdom) # posta a odometria pro cmd a 50hz

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
        self.wait_start = None
        self.last_time = self.get_clock().now().nanoseconds / 1e9
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

        if (self.connection and self.connection.in_waiting > 0):
            data = self.connection.read(self.connection.in_waiting).decode("utf-8")

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
        dt = (current_time - self.last_time) / 1e9 #converts from nanoseconds to seconds

        # Recebe as info da ESP..
        vel_left, vel_right = self.receiveData() # Adicionar mais segurança caso a fila estaj vazia 
        #...
        # Trata os dados pro formato q o cmd_vel usa

        self.current_angular_speed_z = (vel_right - vel_left) / WHEEL_SEPARATION 
        self.current_yaw += self.current_angular_speed_z * dt
        self.current_yaw = self.normalize_angle(self.current_yaw)

        self.current_linear_speed_x = (vel_left + vel_right) / 2 

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
        msg.heaer.stamp           =  current_time.to_msg()
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
        command = f"{vel_left},{vel_right}"
         
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
        

class Walk(Node): #Basicamente me retorna só as velocidades que cada roda deveria estar
    def __init__(self):
        super().__init__('Walk')

        self.cmd_pub =  self.create_publisher(Twist,"/cmd_vel", 10)
        self.odom_sub =  self.create_subscription(Odometry,"/odom", self.odomCallback, 10)
        self.create_timer(0.02, self.movementLoop) #calcula a velocidade com uma frequencia de 50Hz
        
        #Variaveis pra simplificar as intruções
        LEFT = 1
        RIGHT = -1 
        self.instructions = [("FOWARD", 2.0), ("TURN", LEFT), ("FOWARD", 1.0), ("TURN", LEFT), ("FOWARD", 2.0), ("TURN", LEFT), ("FOWARD", 1.0), ("TURN", LEFT)] # Percurso q o robo precisa seguir
        self.instructions_step = 0
        self.setupVariables()


    def setupVariables(self): 
        #   Control
        self.first_run = True
        self.starting_turn = False
        self.starting_walk = True
        self.rate = self.create_rate(50) #50Hz
        self.last_time = 0

        #   Current Position
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.dist_projected = 0
        self.start_x = 0
        self.start_y = 0
        
        #   Target Position 
        self.target_x = 0 # To the sides
        self.target_y = 0 # Foward
        self.target_yaw = 0

        #   Speed
        self.current_linear_speed_x = 0
        self.current_linear_speed_y = 0
        self.current_angular_speed_z = 0 # rad/s

        # PID
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.integral_error = 0
        self.max_integral = 0
        self.last_error = 0.0

    def odomCallback(self, msg): # só atribui os valores as variaveis certas
#        p = msg.pose.pose.position
#        q = msg.pose.pose.orientation
#        t = msg.twist.twist

#        self.get_logger().info(
#            f"[ODOM] pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
#            f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}) "
#            f"v=({t.linear.x:.3f}, {t.linear.y:.3f}) w={t.angular.z:.3f}",
#            throttle_duration_sec=0.5)

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        roll, pitch, yaw = transforms3d.euler.quat2euler((q.w,q.x,q.y,q.z))

        self.current_yaw = yaw
        if(self.first_run): # inicializa esse angulo "zerado" como o padrão
            self.target_yaw = yaw
            self.first_run = False

        #Velocidade de cada roda

        self.target_speed_left = ROBOT_CENTER_SPEED - msg.twist.twist.angular.z * WHEEL_SEPARATION / 2
        self.target_speed_right = ROBOT_CENTER_SPEED + msg.twist.twist.angular.z * WHEEL_SEPARATION / 2


    def movementLoop(self): # ------- OK
        if(self.instructions_step == 8):
            self.instructions_step = 0
        instruction = self.instructions[self.instructions_step][0]
        value = self.instructions[self.instructions_step][1]

#        self.get_logger().info(f"yaw={self.current_yaw:.3f} dist={self.dist_projected:.2f}")

        if(instruction == "FOWARD"):
            if(self.starting_walk == True):
                self.starting_walk = False
                self.start_x = self.current_x
                self.start_y = self.current_y
            self.moveStraight(value)
        
        elif(instruction == "TURN"):
            self.turn_90(value)


    def moveStraight(self, target_distance): # ------- OK # Principalmente acha a velocidade angular que o robo deve ter pra se manter reto
        cmd = Twist()
        cmd.linear.x = ROBOT_CENTER_SPEED 
    
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = (current_time - self.last_time)

        dx = self.current_x - self.start_x 
        dy = self.current_y - self.start_y

        self.dist_projected = dx * math.cos(self.target_yaw) + dy * math.sin(self.target_yaw) #---> Vem da algebra Linear, dessa forma não considera a parte que o robo andou sem ser para o objetivo 

        self.get_logger().info(f"dx={dx:.3f} x={self.current_x:.3f} y={self.current_y:.2f}  dist={self.dist_projected:.2f}  cos={math.cos(self.target_yaw):.2f}")
        if(self.dist_projected >= target_distance):
            self.dist_projected = 0
            self.instructions_step += 1
            self.starting_walk = True
            self.starting_turn = True
            return

        self.last_time = current_time


        # --- AQUI ESTÁ O "DEFEITO" DE SOFTWARE ---
        # Um viés constante simulando que o motor direito/esquerdo tem potências diferentes.
        # Se for 0.0, ele anda reto. Um valor como 0.03 ou 0.05 vai forçar o robô a entortar.
        motor_bias = 0.04  # rad/s (tente valores entre 0.02 e 0.08)

        # O PID calcula a correção, mas o 'motor_bias' empurra o robô para fora da linha
        cmd.angular.z = self.angularPID(self.target_yaw, dt) + motor_bias
        self.cmd_pub.publish(cmd)

    def angularPID(self, target_angle, dt):
        #PID
        error = target_angle - self.current_yaw
        
        # P
        P = self.kp * error

        # I
        self.integral_error += error * dt

        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        I = self.ki * self.integral_error

        # D
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        D = self.kd * derivative

        self.last_error = error
        return (P + I + D) #Velocidade angular z


    def turn_90(self, direction): # ------- OK # direction deve ser +-1 pra significar se é sentido horario ou anti-horario
        #adicionar algum mecanismo para garantir q antes dele girar 90 ele vai estar em cima da linha
        cmd = Twist()
        cmd.linear.x = 0.0
        rotation = direction * math.pi / 2 # roda no sentido horario
        self.current_angular_speed_z = direction * (ROBOT_CENTER_SPEED + ROBOT_CENTER_SPEED) / WHEEL_SEPARATION # com essa velocidade angular ele gira no lugar, supostamente

        if(self.starting_turn):
            self.target_yaw += rotation # adiciona à angulação atual a curva de 90 graus
            self.starting_turn = False
    
        error = math.atan2(math.sin(self.target_yaw - self.current_yaw), # calcula o erro da rotação(msm jeito q calcula o da velocidade)
                           math.cos(self.target_yaw - self.current_yaw))
        if abs(error) > 0.02: # keeps turning until it reaches the target angle
            cmd.angular.z = self.current_angular_speed_z
            self.cmd_pub.publish(cmd)
        else: # para de gira
            self.instructions_step += 1
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Walk())
    rclpy.shutdown()