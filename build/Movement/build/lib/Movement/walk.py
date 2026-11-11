import rclpy
import math
# from tf2_ros import TransformBroadcaster
import transforms3d
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from Movement.robot_constants import *
import time




class Walk(Node): #Basicamente me retorna só as velocidades que cada roda deveria estar
    def __init__(self):
        super().__init__('Walk')

        self.cmd_pub =  self.create_publisher(Twist,"/cmd_vel", 10)
        self.odom_sub =  self.create_subscription(Odometry,"/odom", self.odomCallback, 10)
        self.create_timer(0.02, self.movementLoop) #calcula a velocidade com uma frequencia de 50Hz
        
        #Variaveis pra simplificar as intruções
        LEFT = 1
        RIGHT = -1 
        self.instructions = [("FOWARD", 13.0), 
                             ("TURN", LEFT), 
                             ("FOWARD", 9.0), 
                             ("TURN", LEFT), 
                             ("FOWARD", 13.0), 
                             ("TURN", LEFT), 
                             ("FOWARD", 9.0), 
                             ("TURN", LEFT)] # Percurso q o robo precisa seguir
        self.instructions_step = 0
        self.setupVariables()


    def setupVariables(self): 
        #   Control
        self.first_run = True
        self.starting_turn = False
        self.starting_walk = True
        self.last_time = None

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
        self.kp = 2.0
        self.ki = 0.15
        self.kd = 0.1
        self.integral_error = 0.0
        self.max_integral = 0.5
        self.last_error = 0.0

    def odomCallback(self, msg): # só atribui os valores as variaveis certas
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
        if(instruction == "FOWARD"):
            if(self.starting_walk == True):
                self.starting_walk = False
                self.start_x = self.current_x
                self.start_y = self.current_y
            self.moveStraight(value)
        
        elif(instruction == "TURN"):
            self.turn_90(value)


    def moveStraight(self, target_distance): # ------- OK # Principalmente acha a velocidade angular que o robo deve ter pra se manter reto
        remaining = target_distance - self.dist_projected

        # Rampa: 1.0 longe, cai linearmente até MIN_FACTOR perto do alvo
            
        ACCEL_DIST = 0.5     # metros para chegar na velocidade cheia
        DECEL_DIST = 1.0     # metros antes do alvo para começar a frear
        START_FACTOR = 0.2   # velocidade inicial (20%)
        MIN_FACTOR = 0.15    # piso na chegada

        # Rampa de subida: 0.2 -> 1.0 ao longo dos primeiros 0.5 m
        up = START_FACTOR + (1.0 - START_FACTOR) * (self.dist_projected / ACCEL_DIST)

        # Rampa de descida: 1.0 -> piso nos últimos 1.5 m
        down = remaining / DECEL_DIST

        fator = max(MIN_FACTOR, min(1.0, up, down))

        cmd = Twist()
        cmd.linear.x = ROBOT_CENTER_SPEED * fator
    
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.last_time is None:
            self.last_time = current_time
            return  
        
        dt = (current_time - self.last_time)
        
        dx = self.current_x - self.start_x 
        dy = self.current_y - self.start_y

        self.get_logger().info(f"dx={dx:.3f} dy = {dy:.3f} dist={self.dist_projected:.2f}")


        self.dist_projected = dx * math.cos(self.target_yaw) + dy * math.sin(self.target_yaw) #---> Vem da algebra Linear, dessa forma não considera a parte que o robo andou sem ser para o objetivo 

        #self.get_logger().info(f"dx={dx:.3f} x={self.current_x:.3f} y={self.current_y:.2f}  dist={self.dist_projected:.2f}  cos={math.cos(self.target_yaw):.2f}")
        if(self.dist_projected >= target_distance):
            self.dist_projected = 0
            self.last_error = 0.0
            self.integral_error = 0.0
            self.instructions_step += 1
            self.starting_walk = True
            self.starting_turn = True
            time.sleep(0.2)
            return

        self.last_time = current_time

        # O PID calcula a correção, mas o 'motor_bias' empurra o robô para fora da linha
        cmd.angular.z = self.angularPID(self.target_yaw, dt) 
        self.cmd_pub.publish(cmd)

    def angularPID(self, target_angle, dt):
        #PID
        error = math.atan2(math.sin(target_angle - self.current_yaw),
                           math.cos(target_angle - self.current_yaw))
        # P
        P = self.kp * error

        # I
        if(dt > 0):
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
            self.target_yaw += rotation 
            self.normalize_angle(self.target_yaw)
            self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
            self.starting_turn = False

            self.last_error = 0.0
            self.integral_error = 0.0
            self.last_time = self.get_clock().now().nanoseconds / 1e9

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = (current_time - self.last_time)
        if dt <= 0:
            dt = 0.02 # Fallback de segurança para 50Hz
        self.last_time = current_time
        
        pid_output = self.angularPID(self.target_yaw, dt)

        max_turn_speed = 1.5 # rad/s
        cmd.angular.z = max(min(pid_output, max_turn_speed), -max_turn_speed)

        self.get_logger().info(f"yaw={self.current_yaw:.3f} target_yaw = {self.target_yaw:.3f} dist={self.dist_projected:.2f}")

        error = math.atan2(math.sin(self.target_yaw - self.current_yaw), # calcula o erro da rotação(msm jeito q calcula o da velocidade)
                           math.cos(self.target_yaw - self.current_yaw))
        if abs(error) > 0.017: # keeps turning until it reaches the target angle
            self.cmd_pub.publish(cmd)
        else: # para de gira
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


            self.last_error = 0.0
            self.integral_error = 0.0
            self.instructions_step += 1
            self.starting_walk = True
            self.starting_turn = True
            time.sleep(0.2)
    
    def normalize_angle(self, angle): # Limits the angle between -180 and 180
            while angle > math.pi: angle -= 2.0 * math.pi
            while angle < -math.pi: angle += 2.0 * math.pi
            return angle


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Walk())
    rclpy.shutdown()