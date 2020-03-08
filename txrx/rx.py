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
            tag = tokens[1]

            # Check if tag is downloaded
            port.write('Checking tag {}'.format(tag))
            if monitor.check_image_local(tag):
                port.write('Tag is valid')
            else:
                # If not, check if the tag is valid on dockerhub
                if monitor.check_image(tag):
                    port.write('Tag is available on dockerhub. Use \'pull\' to get the desired image.')
                else:
                    port.write('Tag is invalid')
                port.send_end_of_command()
                continue

            port.write('Starting container. Port closing...')

            try:
                container = monitor.create_container(tag=tag)
            except Exception as ex:
                port.write('Could not start container.')
                port.write(str(ex))
                port.send_end_of_command()
                continue

            # Close the port so the container can use it. Reopen when it closes
            port.close()
            monitor.run_and_monitor(container)
            port.open()
            port.write('Container finished execution. Port opened.')
        elif cmd == 'pull':
            tag = tokens[1]
            
            # Check if tag is in the registry 
            port.write('Checking tag {}'.format(tag))
            if monitor.check_image(tag):
                port.write('Tag is valid')
            else:
                port.write('Tag is invalid')
                port.send_end_of_command()
                continue

            # Pull image
            port.write('Pulling image. This might take a moment...')
            monitor.pull_image(tag)

            # Check if image was successfully pulled
            if monitor.check_image_local(tag):
                port.write('Image pulled successfully.')
            else:
                port.write("Image was not pulled successfully.")

        elif cmd == 'tags':
            tags = monitor.get_tags()
            port.write('Tags currently available:')
            for tag in tags:
                port.write(tag.strip())
        elif cmd == 'clean':
            port.write('Removing all images...')
            monitor.clean()
            port.write('Images successfully removed.')
            pass
        else:
            port.write('Unknown command: {}'.format(cmd))


        port.send_end_of_command()


    port.close()
    monitor.close()

if __name__ == '__main__':
    main()
