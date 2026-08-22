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

        self.get_logger().info("No transformador iniciado, espernado coordenadas da visão.") #debug

    def point_callback(self, msg: PointStamped):
        source_frame = msg.header.frame_id # frame de origem das coordenadas (visao), puxamos da mensagem recebida da visao
        target_frame = 'height_link' # frame de destino (braco)

        try:
            transform = self.tf_buffer.lookup_transform( # pede ao tf2 a trasnformada entre a visao e o braco mais recente
                target_frame, 
                source_frame, 
                rclpy.time.Time())

            transformed_point = tf2_geometry_msgs.do_transform_point(msg, transform) # utiliza a matriz de transformada para converter as coordenadas do cubo recebidas da visao (msg) em relacao ao braco
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
    