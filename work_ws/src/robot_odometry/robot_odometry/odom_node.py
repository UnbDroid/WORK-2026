#importaçoes de bibliotecas e ferramentas

from platform import node

import rclpy                                                  #biblioteca do ROS2 para Python
from rclpy.node import Node                                   #classe base para criar um nó ROS2
from nav_msgs.msg import Odometry                             #mensagem de odometria, que contém informações sobre a posição e velocidade do robô
from geometry_msgs.msg import Quaternion, TransformStamped    #mensagem para representar a orientação do robô em formato de quaternion
from std_msgs.msg import Float32MultiArray                    #desempacotar o array de 4 posições que contém os rad/s de cada motor
import math                                                   
import time

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')                             #inicializa o nó 'odom_node'

        #constantes fisica do robo
        self.R = 0.04
        self.L_x = 0.152
        self.L_y = 0.11315

        #estados iniciais do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()  #variável para armazenar o tempo do último cálculo de odometria


        self.subscription = self.create_subscription(Float32MultiArray, 'wheel_speeds', self.calculate_odometry, 10)  #inscreve-se no tópico 'wheel_speeds'
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

    def calculate_odometry(self, msg: Float32MultiArray):

        #implementacao da lógica para calcular a posição e orientação do robô com base nas velocidades dos motores e no tempo decorrido
        current_time = self.get_clock().now()

        dt = (current_time - self.last_time).nanoseconds / 1e9  #calcula o tempo decorrido desde a última atualização de odometria
        if dt <= 0: return  #evita divisão por zero ou tempos negativos

        #(rad/s) -> (m/s)
        v1, v2, v3, v4 = [w * self.R for w in msg.data]  

        #cinematica direta mecanum
        vx = (v1 + v2 + v3 + v4) / 4
        vy = (-v1 + v2 + v3 - v4) / 4                #sinais negativos para refletir a direção correta do movimento lateral
        wz = (-v1 + v2 - v3 + v4) / (4 * (self.L_x + self.L_y))

        #Integracao (transformação de velocidades para posições)
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt 
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odom(current_time, vx, vy, wz)
        self.last_time = current_time


    def publish_odom(self, current_time, vx, vy, wz):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg() 
        odom_msg.header.frame_id = 'odom'               #frame de referência para a odometria
        odom_msg.child_frame_id = 'base_link'           #frame de referência para o robô 

        #posição do robô
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        #orientação do robô (convertida para quaternion)
        quat = Quaternion()
        quat.z = math.sin(self.theta / 2) #o robo so gira em torno do eixo z, entao os componentes x e y do quaternion sao zero
        quat.w = math.cos(self.theta / 2)
        odom_msg.pose.pose.orientation = quat

        #velocidades lineares e angulares do robô
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        self.odom_publisher.publish(odom_msg)  #publica a mensagem de odometria no tópico '/odom'

def main(args=None):
    rclpy.init(args=args)                                      #inicializa o ROS2
    odom_node = OdomNode()                                     #cria nó de odometria
    rclpy.spin(odom_node)                                      #mantém o nó rodando
    odom_node.destroy_node()
    rclpy.shutdown()                                           #finaliza o ROS2