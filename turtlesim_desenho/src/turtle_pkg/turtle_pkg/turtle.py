import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
import math

class DrawingNode(Node):
    def __init__(self):
        super().__init__('nodo_de_desenho')
        self.publicador = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.spawn_turtle()
        self.set_pen_color(255, 0, 0, 5, 0)

    def spawn_turtle(self):
        self.get_logger().info('Criando tartaruga...')
        cliente = self.create_client(Spawn, '/spawn')
        while not cliente.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço "/spawn" não disponível, aguardando...')
        solicitacao = Spawn.Request()
        solicitacao.x = 4.0
        solicitacao.y = 2.0
        solicitacao.theta = 0.0
        solicitacao.name = 'artista'
        futuro = cliente.call_async(solicitacao)
        rclpy.spin_until_future_complete(self, futuro)
        if futuro.result() is not None:
            self.get_logger().info('Tartaruga criada com sucesso')
        else:
            self.get_logger().error('Falha ao criar a tartaruga')
    
    def kill_turtle(self):
        self.get_logger().info('Matando tartaruga...')
        cliente = self.create_client(Kill, '/kill')
        while not cliente.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço "/kill" não disponível, aguardando...')
        solicitacao = Kill.Request()
        solicitacao.name = 'artista'
        futuro = cliente.call_async(solicitacao)
        rclpy.spin_until_future_complete(self, futuro)
        if futuro.result() is not None:
            self.get_logger().info('Tartaruga morta com sucesso')
        else:
            self.get_logger().error('Falha ao matar a tartaruga')

    def set_pen_color(self, r, g, b, largura, off):
        self.get_logger().info('Configurando a cor da caneta...')
        cliente = self.create_client(SetPen, '/turtle1/set_pen')
        while not cliente.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço "/turtle1/set_pen" não disponível, aguardando...')
        solicitacao = SetPen.Request()
        solicitacao.r = r
        solicitacao.g = g
        solicitacao.b = b
        solicitacao.width = largura
        solicitacao.off = off
        futuro = cliente.call_async(solicitacao)
        rclpy.spin_until_future_complete(self, futuro)
        if futuro.result() is not None:
            self.get_logger().info('Cor da caneta configurada com sucesso')
        else:
            self.get_logger().error('Falha ao configurar a cor da caneta')

    def draw_circle(self):
        raio = 1.0
        velocidade_angular = 0.5
        distancia = 2 * math.pi * raio
        duracao = distancia / velocidade_angular
        mensagem_velocidade = Twist()
        mensagem_velocidade.linear.x = float(velocidade_angular * raio)
        mensagem_velocidade.angular.z = float(velocidade_angular)
        t0 = self.get_clock().now().to_msg()
        while (self.get_clock().now().to_msg().sec - t0.sec) < duracao:
            self.publicador.publish(mensagem_velocidade)
            rclpy.spin_once(self, timeout_sec=0.01)
        mensagem_velocidade.linear.x = float(0)
        mensagem_velocidade.angular.z = float(0)
        self.publicador.publish(mensagem_velocidade)

def main(args=None):
    rclpy.init(args=args)
    dn = DrawingNode()
    for _ in range(3):
        dn.draw_circle()
    dn.kill_turtle()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
