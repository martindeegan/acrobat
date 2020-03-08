# Serial port for receiving commands from control station through the SiK radio modules

import signal
import sys

from serial_port import SerialPort
from docker_monitor import DockerMonitor

def main():
    port = SerialPort("/dev/acrobat/radio")
    monitor = DockerMonitor()

    while True:
        user_input = port.read()
        tokens = user_input.split()

        if len(tokens) == 0:
            continue

        cmd = tokens[0]

        print('Received command {}'.format(user_input))
        if cmd == 'run':
            port.send_ack()

            try:
                image = monitor.pull_image_if_not_exists(tokens[1])
                container = monitor.create_container(image)
                port.send_success()
            except Exception as ex:
                print(str(ex))
                port.send_failed()
                continue

            # Close the port so the container can use it. Reopen when it closes
            port.close()
            monitor.run_and_monitor(container)
            port.open()



    port.close()
    monitor.close()

if __name__ == '__main__':
    main()
