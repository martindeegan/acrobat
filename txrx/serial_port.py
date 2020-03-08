import serial
import socket

class SerialPort:
    def __init__(self, serial_port: str):
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

    def send_ack(self):
        self.write('ack')

    def check_ack(self, tries=1):
        for i in range(tries):
            if 'ack' in self.read():
                return True
        return False

    def send_success(self):
        self.write('success')

    def send_failed(self):
        self.write('failed')

    def check_succeeded(self, tries=3):
        for i in range(tries):
            line = self.read()
            if 'success' in line:
                return True
            elif 'failed' in line:
                return False
        return False

    def handshake(self):
        print('Waiting for handshake.')

        name = socket.gethostname()
        ack = False
        other_name = None
        other_ack = False

        while (not ack) and (not other_ack):
            if not other_ack:
                self.write('handshake {}'.format(name))
            line = self.read()
            tokens = line.split()
            if len(tokens) > 0:
                if tokens[0] == 'handshake':
                    other_name = tokens[1]
                    self.write('ack')
                    ack = True
                elif tokens[0] == 'ack':
                    other_ack = True
            
        print('Connected with {}'.format(other_name))
