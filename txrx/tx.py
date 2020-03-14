# Serial CLI for talking with the jetson hardware through the SiK radio modules
import signal
import sys
import readline

from serial_port import SerialPort
from docker_monitor import DockerMonitor

global tags
global commands
commands = []


def signal_handler(signal, frame):
    pass


def print_command_help(cmd, args, description):
    global commands
    commands.append(cmd)
    msg = cmd + ' '
    for arg in args:
        msg += '<{}>'.format(arg)
    msg = msg.ljust(30)
    msg += description
    print(msg)


def print_help(args=None):
    print('===========================================================================================================')
    print('Commands:')
    print_command_help('run', [
                       'tag:optional'], 'Runs the given docker tag. If tag is not provided, runs :latest.')
    print_command_help('pull', [
                       'tag:optional'], 'Pulls the given docker tag. If tag is not provided, pull :latest.')
    print_command_help('clean', [], 'Removes all images from drone.')
    print_command_help('tags', [], 'Lists all tags available to run.')
    print_command_help('reboot', [], 'Restarts the acrobat computer.')
    print_command_help('exit', [], 'Exits TX.')
    print_command_help('help', [], 'Print this message.')
    print('===========================================================================================================')


def completer(text, state):
    global commands

    options = [cmd for cmd in commands if cmd.startswith(text)]

    if state < len(options):
        return options[state]
    else:
        return None


def wait_for_end_of_message_or_print(port):
    while True:
        msg = port.read()
        if port.end_of_transmission_message in msg:
            break
        elif msg == '':
            continue
        else:
            print(msg.strip())


def main(serial_id):
    port = SerialPort(serial_id)
    monitor = DockerMonitor()

    signal.signal(signal.SIGINT, signal_handler)
    print('Welcome to TX')
    print_help()

    readline.parse_and_bind("tab:complete")
    readline.set_completer(completer)
    while True:
        user_input = input('#: ')
        tokens = user_input.split()
        global commands
        commands = commands + monitor.get_tags()

        if len(tokens) == 0:
            continue

        cmd = tokens[0]

        if cmd == 'help':
            print_help()
        elif cmd == 'tags':
            port.write(cmd)
        elif cmd == 'exit':
            break
        elif cmd == 'run':
            if len(tokens) < 2:
                tag = 'latest'
            else:
                tag = tokens[1]
            port.write(cmd + ' ' + tag)
        elif cmd == 'pull':
            if len(tokens) < 2:
                tag = 'latest'
            else:
                tag = tokens[1]
            port.write(cmd + ' ' + tag)
        elif cmd == 'clean':
            port.write(cmd)
        elif cmd == 'reboot':
            port.write(cmd)
        else:
            print('Unknown command: {}'.format(cmd))
            continue

        # Wait for and print all messages received from rx
        wait_for_end_of_message_or_print(port)

    # Close port on exit
    port.close()
    monitor.close()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Need to pass the device ID in')
        exit()

    main(sys.argv[1])
