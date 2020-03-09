import serial


class SerialPort:
    def __init__(self, serial_port: str):
        """ Initializes the serial port. Requires the device path. """
        self.port = serial.Serial(serial_port, 115200, timeout=3)
        print('Connected to serial port: {}'.format(self.port.name))

    def open(self):
        print('Opening serial port {}'.format(self.port.name))
        self.port.open()

    def close(self):
        print('Closing serial port {}'.format(self.port.name))
        self.port.close()

    def write(self, msg: str):
        escaped_msg = msg + '\n'
        self.port.write(escaped_msg.encode())

    def read(self):
        line_bytes = self.port.readline()
        return str(line_bytes, encoding='utf-8')

    @property
    def end_of_transmission_message(self):
        return 'END OF TRANSMISSION'

    def send_end_of_command(self):
        self.write(self.end_of_transmission_message)
