import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from datetime import datetime
import serial
import struct
from math import copysign, pi
from sensor_msgs.msg import JointState

class HwClass(Node):

    def __init__(self):
        # Call the constructor of the parent class
        super().__init__('hwinterface')

        # Get the publish rate for encoder readings from the ROS 2 parameter server
        self.read_encoder_freq = 6

        # Initialize variables for twist timeout and encoder parameters
        self.lastTwistTime = 0
        self.twistTimeout = 5
        self.pasos_vuelta = 356.3 * 2
        self.kdato = 36.28

        # Open a serial connection to the Arduino device
        self.arduino = serial.Serial('/dev/arduino', 115200)

        # Read and discard the first line of data from the Arduino device
        data = self.arduino.readline()

        # Read the current encoder values
        self.steps_enc_left, self.time_enc_left, self.steps_enc_right, self.time_enc_right = self._read_encoder()

        # Create a ROS 2 publisher for wheel state information
        self._name_publisher = "wheel_state"
        self._wheel_pub = self.create_publisher(JointState, self._name_publisher, 10)
        self._timer = self.create_timer(self.read_encoder_freq, self._publisher_callback)

        # Create a ROS 2 subscriber for wheel velocity commands
        self._name_subscriber = "cmd_wheel"
        self.create_subscription(JointState, self._name_subscriber, self._callback_vel, 1)

    def _read_encoder(self):

        # Send the 'N' character to the serial port to request encoder data
        self.arduino.write(str('N').encode())

        # Read the data from the serial port
        # Read the first line, but discard the data
        self.arduino.readline()

        # Read the left encoder steps and time values
        steps_enc_left = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_left = struct.unpack('i', self.arduino.read(4))[0]

        # Read the right encoder steps and time values
        steps_enc_right = struct.unpack('i', self.arduino.read(4))[0]
        time_enc_right = struct.unpack('i', self.arduino.read(4))[0]

        # Read the final line, but discard the data
        self.arduino.readline()

        return steps_enc_left, time_enc_left, steps_enc_right, time_enc_right

    def _publisher_callback(self):

        # Check if it is still receiving data
        self._check_alive()

        # Read the encoder data
        steps_enc_left, time_enc_left, steps_enc_right, time_enc_right = self._read_encoder()

        # Calculate the elapsed time and distance travelled since the last measurement for the right wheel
        dt_right = float((time_enc_right - self.time_enc_right) * (10 ** -6))
        dsteps_right = float(steps_enc_right - self.steps_enc_right)

        # Update the last measurement values for the right wheel
        self.time_enc_right = time_enc_right
        self.steps_enc_right = steps_enc_right

        # Calculate the position and velocity for the right wheel
        posD = (steps_enc_right / self.pasos_vuelta) * 2 * pi
        wd = ((dsteps_right / self.pasos_vuelta) * 2 * pi) / dt_right

        # Publish the right wheel position and velocity information
        self._send_message('RIGHT', posD, wd)

        # Calculate the elapsed time and distance travelled since the last measurement for the left wheel
        dt_left = float((time_enc_left - self.time_enc_left) * (10 ** -6))
        dsteps_left = float(steps_enc_left - self.steps_enc_left)

        # Update the last measurement values for the left wheel
        self.time_enc_left = time_enc_left
        self.steps_enc_left = steps_enc_left

        # Calculate the position and velocity for the left wheel
        posI = (steps_enc_left / self.pasos_vuelta) * 2 * pi
        wi = ((dsteps_left / self.pasos_vuelta) * 2 * pi) / dt_left

        # Publish the left wheel position and velocity information
        self._send_message('LEFT', posI, wd)

        # Print velocity information
        self.get_logger().info(f" {wi}, {wd}")

    def _send_message(self, name, pos, w):
        # Create a new JointState message object
        msg = JointState()

        # Set the values of the JointState message object
        msg.name = [name]  # Set the name of the joint (LEFT or RIGHT)
        msg.position = [pos]  # Set the position value in radians
        msg.velocity = [w]  # Set the velocity value in radians per second
        msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp

        # Publish the JointState message to the 'wheel_state' topic
        self._wheel_pub.publish(msg)

    def _check_alive(self):
        # Get the current time in seconds
        current_time = datetime.now().second

        # Check if the time since the last twist message exceeds the timeout value
        if (current_time - self.lastTwistTime) > self.twistTimeout:
            # Send a "?" character to the Arduino device to request new data
            self.arduino.write(str('?').encode())

            # Log a message indicating that the robot has stopped due to not receiving data
            self.get_logger().info("Parada por no recibir datos")

    def _callback_vel(self, msg):
        # Define a lambda function to get the sign of a number
        sign = lambda x: int(copysign(1, x))

        # Extract the desired angular velocities from the JointState message
        comandD = msg.velocity[0]  # rad/s
        comandI = msg.velocity[1]  # rad/s

        # Convert the angular velocities from rad/s to an integer data value in the range 0-127
        datod = int(round(comandD * self.kdato))
        datoi = int(round(comandI * self.kdato))

        # Limit the data value to the range 10-127 and adjust the sign if necessary
        datod = min(127, max(10 * sign(datod), datod)) if datod != 0 else 0
        datoi = min(127, max(10 * sign(datoi), datoi)) if datoi != 0 else 0

        # Add an offset of 256 to the data value if it is negative
        datod = 256 + datod if datod < 0 else datod
        datoi = 256 + datoi if datoi < 0 else datoi

        # Log the data values (for debugging purposes)
        self.get_logger().info(f"datod: {datod} y datoi: {datoi}")

        # Send the data values to the Arduino device in the format 'V###' using a serial connection
        self.arduino.write(('V' + format(int(datod), '03d') + format(int(datoi), '03d')).encode())

        # Update the last twist time to the current time
        self.lastTwistTime = datetime.now().second


def main(args=None):
    # Initialize the ROS2 system with the given arguments
    rclpy.init(args=args)

    # Create an instance of the HwClass node implementation
    minimal_subscriber = HwClass()

    # Start the ROS2 event loop and wait for it to stop
    rclpy.spin(minimal_subscriber)

    # Clean up the node resources when the event loop is stopped
    minimal_subscriber.destroy_node()

    # Shut down the ROS2 system
    rclpy.shutdown()


if __name__ == '__main__':
    main()
