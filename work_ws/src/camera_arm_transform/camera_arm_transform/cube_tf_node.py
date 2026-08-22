import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import tf2_geometry_msgs

class CubeTransformNode(Node):
    def __init__(self):
        super().__init__('cube_transformer')

        self.tf_buffer = Buffer() # cria um buffer para armazenar coordenadas
        self.tf_listener = TransformListener(self.tf_buffer, self) # cria um listener que preenche o buffer com as transformadas 

        self.subscription = self.create_subscription( # inscreve o no no topico de coordenadas do cubo em relacao ao braco
            PointStamped,
            'cube_coordinates',
            self.point_callback,
            10
        )

        self.publisher = self.create_publisher( # cria o publsisher que vai enviar pra esp as coordenadas do braco em relacao ao cubo
            PointStamped,
            'cube_arm_coordinates',
            10
        )

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.threshold = 0.01

        self.get_logger().info("No transformador iniciado, espernado coordenadas da visão.") #debug

    def point_callback(self, msg: PointStamped):
        source_frame = msg.header.frame_id # frame de origem das coordenadas (visao), puxamos da mensagem recebida da visao
        target_frame = 'height_link' # frame de destino (braco)

        try:
            transform = self.tf_buffer.lookup_transform( # pede ao tf2 a trasnformada entre a visao e o braco mais recente
                target_frame, 
                source_frame, 
                rclpy.time.Time())
            transformed_point = tf2_geometry_msgs.do_transform_point(msg, transform) # cria a mensagem que vai enviar as coordenadas do cubo transformadas para a esp

            # to mudando essa parte aqui pra tirar ruidos na movimentacao do braco, pra esp nao ficar confusa recebendo muitas coordenadas muito proximas, mas ainda diferentes na precisao do double
            # verifica a diferenca entre as coordenadas atuais e as anteriores
            novo_x = transformed_point.point.x
            novo_y = transformed_point.point.y
            novo_z = transformed_point.point.z
            diff_x = abs(novo_x - self.last_x)
            diff_y = abs(novo_y - self.last_y)
            diff_z = abs(novo_z - self.last_z)

            # se a diferenca for menor que um centimetro em todos os eixos nao envia pra esp
            if diff_x < self.threshold and diff_y < self.threshold and diff_z < self.threshold:
                return 

            # atualiza a coordenadas mais recentes
            self.last_x = novo_x
            self.last_y = novo_y
            self.last_z = novo_z

            transformed_point.header.frame_id = target_frame # explicitamos que as coordenadas transformadas estao em relacao ao braco

            self.publisher.publish(transformed_point) # publica as coordenadas do cubo em relacao ao braco no topico para a esp receber
            self.get_logger().info(f'Transformed point: {transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z} in frame {target_frame}') #debug

        except TransformException as ex:
            self.get_logger().error(f'Could not transform point from {source_frame} to {target_frame}: {ex}') #debug

def main(args=None):
    rclpy.init(args=args)
    node = CubeTransformNode() # cria o no que vai fazer a transformada

    # executa o no ate alguem apertar ctrl c
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    