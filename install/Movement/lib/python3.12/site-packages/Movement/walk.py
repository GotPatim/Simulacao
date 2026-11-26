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
                             ("TURN", LEFT),
                             ("STOP", None)] # Percurso q o robo precisa seguir
        self.instructions_step = 0
        self.setupVariables()


    def setupVariables(self): 
        #   Control
        self.first_run = True
        self.starting_turn = False
        self.starting_walk = True
        self.last_time = None
        self.finished = False

        #   Current Position
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        self.dist_projected = 0
        self.start_x = 0
        self.start_y = 0

        #Turn
        self.yaw_vel = 0.8
        self.curve_radius = 0.0
        self.turn_approx = 89.0      # graus, para antes de passar
        self.pause_until = None
        
        #   Target Position 
        self.target_x = 0 
        self.target_y = 0 
        self.target_yaw = 0

        #   Speed
        self.current_linear_speed_x = 0
        self.current_linear_speed_y = 0
        self.current_angular_speed_z = 0 # rad/s

        # Desvio Lateral
        self.ref_x = None      # canto ideal atual do retângulo
        self.ref_y = None
        self.segment_len = 0.0
        self.k_ct = 0.8        # ganho do erro lateral
        self.max_ct_angle = 0.4  # rad, limite da correção

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
        if self.first_run:
            self.target_yaw = yaw
            self.ref_x = self.current_x
            self.ref_y = self.current_y
            self.first_run = False

        #Velocidade de cada roda

        self.target_speed_left = ROBOT_CENTER_SPEED - msg.twist.twist.angular.z * WHEEL_SEPARATION / 2
        self.target_speed_right = ROBOT_CENTER_SPEED + msg.twist.twist.angular.z * WHEEL_SEPARATION / 2


    def movementLoop(self): # ------- OK
        now = self.get_clock().now().nanoseconds / 1e9
        if self.pause_until is not None:
            if now < self.pause_until:
                return
            self.pause_until = None
        
        if self.first_run:
            return

        instruction = self.instructions[self.instructions_step][0]
        value = self.instructions[self.instructions_step][1]
        if(instruction == "FOWARD"):
            if(self.starting_walk == True):
                self.starting_walk = False
                self.start_x = self.current_x
                self.start_y = self.current_y

            if self.instructions_step == 0:
                discount = self.curve_radius 
            else:
                discount = 2 * self.curve_radius
            self.segment_len = value
            self.moveStraight(value - discount)
        
        elif(instruction == "TURN"):
            self.turn_90(value)
        elif(instruction == "STOP"):
            self.cmd_pub.publish(Twist())   # velocidade zero
            if not self.finished:
                self.finished = True
                self.get_logger().info("Fim do Percurso")



    def moveStraight(self, target_distance): # ------- OK # Principalmente acha a velocidade angular que o robo deve ter pra se manter reto
        remaining = target_distance - self.dist_projected

        # Rampa: 1.0 longe
            
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
        
        dx = self.current_x - self.ref_x
        dy = self.current_y - self.ref_y


        self.dist_projected = dx * math.cos(self.target_yaw) + dy * math.sin(self.target_yaw) #---> Vem da algebra Linear, dessa forma não considera a parte que o robo andou sem ser para o objetivo 

        #self.get_logger().info(f"dx={dx:.3f} x={self.current_x:.3f} y={self.current_y:.2f}  dist={self.dist_projected:.2f}  cos={math.cos(self.target_yaw):.2f}")
        if self.dist_projected >= target_distance:
            self.dist_projected = 0
            self.starting_turn = True
            self.ref_x += self.segment_len * math.cos(self.target_yaw)
            self.ref_y += self.segment_len * math.sin(self.target_yaw)
            self.finishSegment(0.5)
            return

        self.last_time = current_time

        # erro lateral em relação à linha ideal (positivo = à esquerda)
        lat = (-(self.current_x - self.ref_x) * math.sin(self.target_yaw)
               + (self.current_y - self.ref_y) * math.cos(self.target_yaw))
        correction = max(-self.max_ct_angle,
                         min(self.max_ct_angle, math.atan(self.k_ct * lat)))

        cmd.angular.z = self.angularPID(self.target_yaw - correction, dt)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"lat={lat:.3f} dist={self.dist_projected:.2f}")

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


    def turn_90(self, direction):  # direction: +1 esquerda, -1 direita
        if self.starting_turn:
            self.target_yaw = self.normalize_angle(
                self.target_yaw + direction * math.pi / 2)
            self.starting_turn = False

        error = self.normalize_angle(self.current_yaw - self.target_yaw)
        if abs(error) < 0.03:
            self.finishSegment(1.0)
            return

        omega = direction * self.yaw_vel
        cmd = Twist()
        cmd.linear.x = abs(omega * self.curve_radius) 
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


    def normalize_angle(self, angle): # Limits the angle between -180 and 180
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def finishSegment(self, pause):
        self.cmd_pub.publish(Twist())   # zera a velocidade
        self.pause_until = self.get_clock().now().nanoseconds / 1e9 + pause
        self.instructions_step += 1
        self.last_error = 0.0
        self.integral_error = 0.0
        self.last_time = None           # evita dt gigante na próxima reta
        self.starting_walk = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Walk())
    rclpy.shutdown()

