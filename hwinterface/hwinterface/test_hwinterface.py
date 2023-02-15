import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

class HwClass(Node):
    def __init__(self):
        super().__init__('hwinterface')

        #self.read_encoder_freq = self.get_parameter('/diff_drive_controller/publish_rate').value
        self.read_encoder_freq = 0.5

        #self.arduino = serial.Serial('/dev/arduino', 115200)
        #data = self.arduino.readline()
        #self.get_logger().info(data)

        # Create ROS2 publisher wheel state
        self._name_publisher = "wheel_state"

        self._wheel_pub = self.create_publisher(JointState, self._name_publisher, 10)
        self._timer = self.create_timer(self.read_encoder_freq, self._publisher_callback)

        #create the subscriber
        #self.create_subscription(JointState, "cmd_wheel", self.callback_vel, 1)


    def _publisher_callback(self):

        msg = JointState()
        #msg.data = 'Hello World: %d' % self.i

        self._wheel_pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    """
    def dum_first_read(self):
        self.get_logger().info("LECTURA DUMMY")
        #send the N letter to the serial port
        self.arduino.write(str('N').encode())
        
        #comienzo = self.arduino.readline()
        steps_enc_left = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_left = struct.unpack('i', self.arduino.read(4))[0]
        steps_enc_right = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i', self.arduino.read(4))[0]
        #final = self.arduino.readline()
        self.time_enc_right_last = time_enc_right
        self.steps_enc_right_last = steps_enc_right
        self.time_enc_left_last = time_enc_left
        self.steps_enc_left_last = steps_enc_left
        self.get_logger().info(str(steps_enc_right))

        self.steps_enc_left_last = struct.unpack('i', self.arduino.read(4))[0]
        self.time_enc_left_last = struct.unpack('i', self.arduino.read(4))[0]
        self.steps_enc_right_last = struct.unpack('i', self.arduino.read(4))[0]
        self.time_enc_right_last = struct.unpack('i', self.arduino.read(4))[0]

    def check(self):
        current_time = self.get_clock_ros().now().nanoseconds
        if (current_time - self.lastTwistTime) > self.twistTimeout * 1e9:
            #send ? to the serial port
            self.arduino.write(str('?').encode())
            self.get_logger().info("Parada por no recibir datos")

    def callback_vel(self, msg):
        sign = lambda x: int(copysign(1, x))
        # Save variables
        comandD = msg.velocity[self.right]  # rad/s
        comandI = msg.velocity[self.left]  # rad/s

        # Convert from rad/s to data 0-127
        datod = int(round(comandD * self.kdato))
        datoi = int(round(comandI * self.kdato))

      
        # Saturation of data
        if abs(datod) > 127:
            datod = 127 * sign(datod)
        if abs(datoi) > 127:
            datoi = 127 * sign(datoi)

        # Minimum velocity assigned
        if (abs(datod) < 10 and datod != 0):
            datod = 10 * sign(datod)
        if (abs(datoi) < 10 and datoi != 0):
            datoi = 10 * sign(datoi)

        # Negative velocity convert
        if datod < 0:
            datod = 256 + datod
        if datoi < 0:
            datoi = 256 + datoi
       
        datod = min(127, max(10 * sign(datod), datod)) if datod != 0 else 0
        datoi = min(127, max(10 * sign(datoi), datoi)) if datoi != 0 else 0
        datod = 256 + datod if datod < 0 else datod
        datoi = 256 + datoi if datoi < 0 else datoi

        # TODO: Analizar si se debe enviar algun mecanismo de eco
        self.arduino.write(('V' + format(int(datod), '03d') +
                            format(datoi, '03d')).encode())

        # Se toma para ver si hay un Timeout.
        #self.lastTwistTime = self.get_clock().now().seconds


    def read_encoders(self):

        self.arduino.write(str('N').encode())
        comienzo = self.arduino.readline()
        steps_enc_left = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_left = struct.unpack('i', self.arduino.read(4))[0]
        steps_enc_right = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i', self.arduino.read(4))[0]
        final = self.arduino.readline()
        data = JointState()

        dt_right = float((time_enc_right - self.time_enc_right_last) * (10 ** -6))
        dsteps_right = float(steps_enc_right - self.steps_enc_right_last)
        self.time_enc_right_last = time_enc_right
        self.steps_enc_right_last = steps_enc_right

        posD = (steps_enc_right / self.pasos_vuelta) * 2 * self.pi
        wd = ((dsteps_right / self.pasos_vuelta) * 2 * self.pi) / dt_right

        data.name = ["RIGHT"]
        data.position = [posD]
        data.velocity = [wd]
        data.effort = [0]
        data.header.stamp = self.get_clock_ros().now().to_msg()
        data.header.frame_id = "base_link"
        self.data_pub.publish(data)

        dt_left = float((time_enc_left - self.time_enc_left_last) * (10 ** -6))
        dsteps_left = float(steps_enc_left - self.steps_enc_left_last)

        self.time_enc_left_last = time_enc_left
        self.steps_enc_left_last = steps_enc_left

        posI = (steps_enc_left / self.pasos_vuelta) * 2 * self.pi
        wi = ((dsteps_left / self.pasos_vuelta) * 2 * self.pi) / dt_left

        data = JointState()
        data.name = ["LEFT"]
        data.position = [posI]
        data.velocity = [wi]
        data.effort = [0]
        data.header.stamp = self.node.get_clock().now().to_msg()
        data.header.frame_id = "base_link"

        #publish topic
        self.data_pub.publish(data)
        self.get_logger().info("WI: %.2f   WD: %.2f" % (wi, wd))
    """


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = HwClass()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()