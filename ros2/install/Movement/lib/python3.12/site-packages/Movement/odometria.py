import rclpy
import math
from tf2_ros import TransformBroadcaster
import transforms3d
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from Movement.robot_constants import *


class HWInterface(Node): #Refazer 
    def __init__(self):
        super().__init__('HWInterface')

        self.cmd_pub =  self.create_subscription(Twist,"/cmd_vel", self.sendSpeed, 10)
        self.odom_pub =  self.create_publisher(Odometry,"/odom", 50)

        self.setupVariables()
        self.create_timer(0.02, self.calcOdom) # posta a odometria pro cmd a 50hz

    def setupVariables(self):
        #   Control
        self.last_time = self.get_clock().now()

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

    def calcOdom(self): 
        # Recebe a velocidade dos motores e calcula: 
        # velocidade_linear_x ( velocidade do centro do robo na direção x)
        # velocidade_linear_y (velocidade do centro no eixo y)
        # velocidade angular ( velocidade com a qual o robo está virando)  
        # posicao_x (posição atual dps de andar um pouco em x )  
        # posicao_y (posição atual dps de andar um pouco em y )
        # Angulo que o robo está virado em relação aos eixos x e y

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 #converts from nanoseconds to seconds

        # Recebe as info da ESP..
        vel_left = 0
        vel_right = 0
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
    
    def sendSpeed(self):
        #manda a velocidade calculada pra ESP
        return 
        

class Walk(Node): #Basicamente me retorna só as velocidades que cada roda deveria estar
    def __init__(self):
        super().__init__('Walk')

        self.cmd_pub =  self.create_publisher(Twist,"/cmd_vel", 10)
        self.odom_sub =  self.create_subscription(Odometry,"/odom", self.odomCallback)
        self.create_timer(0.02, self.movementLoop) #calcula a velocidade com uma frequencia de 50Hz
        
        #Variaveis pra simplificar as intruções
        LEFT = 1
        RIGHT = -1 
        self.instructions = [("FOWARD", 2.0), ("TURN", LEFT), ("FOWARD", 1.0)] # Percurso q o robo precisa seguir
        self.instructions_step = 0
        self.setupVariables()


    def setupVariables(self): 
        #   Control
        self.first_run = True
        self.starting_turn = False
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
        self.current_x = msg.pose.pose.position.x
        self.current_x = msg.pose.pose.position.y

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
        instruction = self.instructions[self.instructions_step][0]
        value = self.instructions[self.instructions_step][1]

        if(instruction == "FOWARD"):
            if(self.dist_projected == 0):
                self.start_x = self.current_x
                self.start_y = self.current_y
            self.moveStraight(value)
        
        elif(instruction == "TURN"):
            self.turn_90(value)


    def moveStraight(self, target_distance): # ------- OK # Principalmente acha a velocidade angular que o robo deve ter pra se manter reto
        cmd = Twist()
        cmd.linear.x = ROBOT_CENTER_SPEED 
    
        current_time = self.get_clockcurrent_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        dx = self.current_x - self.start_x 
        dy = self.current_y - self.start_y

        self.dist_projected = dx * math.cos(self.target_yaw) + dy * math.sin(self.target_yaw) #---> Vem da algebra Linear, dessa forma não considera a parte que o robo andou sem ser para o objetivo 

        if(self.dist_projected >= target_distance):
            self.dist_projected = 0
            self.instructions_step += 1
            self.starting_turn = True
            return

        self.last_time = current_time

        cmd.angular.z = self.angularPID(self.target_yaw, dt)
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
        derivative = (error - self.last_error) / dt
        D = self.kd * derivative

        self.last_error = error
        return (P + I + D) #Velocidade angular z


    def turn_90(self, direction): # ------- OK # direction deve ser +-1 pra significar se é sentido horario ou anti-horario
        #adicionar algum mecanismo para garantir q antes dele girar 90 ele vai estar em cima da linha
        cmd = Twist()
        cmd.linear.x = 0
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
            cmd.angular.z = 0
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Walk())
    rclpy.shutdown()