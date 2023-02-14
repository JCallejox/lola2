import rclpy
import serial,time 
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState

#Declaramos las constantes que utilizaremos para inicializar

class HWclass:
    def __init__(self):
        #Dividiendo los steps dados por el encoder por pasos_vuelta hayamos el numero de vueltas
        self.pasos_vuelta = 356.3 * 2
        self.left = 0
        self.right = 0
        self.k_dato = 36.28
        self.twist_timeout = 20
        self.read_encoder_frequency = None

        # Create node
        self.node = rclpy.create_node("HWclass")
        self.self.read_encoder_freq = node.get_parameter('/diff_drive_controller/publish_rate').value
        if self.read_encoder_freq is None:
            node.declare_parameter('/diff_drive_controller/publish_rate', 6)
            publish_rate = 6

        self.arduino = serial.Serial('/dev/arduino', 115200)
        time.sleep(0.01)


    def __del__(self):
        self.arduino.write(str('?').encode())
        self.arduino.close()
    # Public





    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()