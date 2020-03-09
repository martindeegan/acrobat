# Serial port for receiving commands from control station through the SiK radio modules

import time
from serial_port import SerialPort
from docker_monitor import DockerMonitor


def exec_pull(port, monitor, tokens):
    tag = tokens[1]

    # Check if tag is in the registry
    port.write('Checking tag {}'.format(tag))
    if monitor.check_image(tag):
        port.write('Tag is valid')
    else:
        port.write('Tag is invalid')
        return

    # Pull image
    port.write('Pulling image. This might take a moment...')
    monitor.pull_image(tag)

    # Check if image was successfully pulled
    if monitor.check_image_local(tag):
        port.write('Image pulled successfully.')
    else:
        port.write("Image was not pulled successfully.")


def run_and_monitor(self, port, container):
    """Runs a container and monitors it until it exits. Blocks until container exits."""
    container.start()
    port.write(
        '====================================================================')
    port.write('Launching container {}, id: {}'.format(
        container.image, container.id))
    port.write('Monitoring container...')
    container.wait()
    port.write('Container finished.')

    # Remove the container so we can start another with name "acrobat"
    container.remove()


def exec_run(port, monitor, tokens):
    tag = tokens[1]

    # Check if tag is downloaded
    port.write('Checking tag {}'.format(tag))
    if monitor.check_image_local(tag):
        port.write('Tag is valid')
    else:
        # If not, check if the tag is valid on dockerhub
        if monitor.check_image(tag):
            port.write(
                'Tag is available on dockerhub. Use \'pull\' to get the desired image.')
        else:
            port.write('Tag is invalid')

        return

    port.write('Starting container. Port closing...')

    try:
        container = monitor.create_container(tag=tag)
    except Exception as ex:
        port.write('Could not start container.')
        port.write(str(ex))
        return

    # Close the port so the container can use it. Reopen when it closes
    port.close()
    monitor.run_and_monitor(container)
    port.open()
    port.write('Container finished execution. Port opened.')


def exec_tags(port, monitor):
    tags = monitor.get_tags()
    port.write('Tags currently available:')
    for tag in tags:
        port.write(tag.strip())


def exec_clean(port, monitor):
    port.write('Removing all images...')
    monitor.clean()
    port.write('Images successfully removed.')


def exec_reboot(port, monitor):
    port.write('Rebooting')
    port.send_end_of_command()
    exit()


def main():
    port = SerialPort("/dev/acrobat/radio")
    monitor = DockerMonitor()

    print("Sending start up lights")
    for _ in range(5):
        port.write(
            "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        time.sleep(1)
    print("Done. Ready for user input")

    while True:
        user_input = port.read()
        tokens = user_input.split()

        if len(tokens) == 0:
            continue

        cmd = tokens[0]

        print('Received command {}'.format(user_input))
        if cmd == 'run':
            exec_run(port, monitor, tokens)
        elif cmd == 'pull':
            exec_pull(port, monitor, tokens)
        elif cmd == 'tags':
            exec_tags(port, monitor)
        elif cmd == 'clean':
            exec_clean(port, monitor)
        elif cmd == 'reboot':
            exec_reboot(port, monitor)
        else:
            port.write('Unknown command: {}'.format(cmd))

        # End of command execution
        port.send_end_of_command()

    port.close()
    monitor.close()


if __name__ == '__main__':
    main()
