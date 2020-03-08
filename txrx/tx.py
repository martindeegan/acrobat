# Serial CLI for talking with the jetson hardware through the SiK radio modules
import signal
import sys
import readline

from serial_port import SerialPort
from docker_monitor import DockerMonitor

def signal_handler(signal, frame):
    pass

def print_command_help(cmd, args, description):
    msg = cmd + ' '
    for arg in args:
        msg += '<{}>'.format(arg)
    msg = msg.ljust(30)
    msg += description
    print(msg)


def print_help(args=None):
    print('===========================================================================================================')
    print('Commands:')
    print_command_help('run', ['tag:optional'], 'Runs the given docker tag. If tag is not provided, runs :latest_arm64.')
    print_command_help('tags', [], 'Lists all tags available to run.')
    print_command_help('exit', [], 'Exits TX.')
    print_command_help('help', [], 'Print this message.')
    print('===========================================================================================================')

global tags
global commands
commands = ['exit', 'help', 'tags', 'run ']

def completer(text, state):
    global commands
    
    options = [cmd for cmd in commands if cmd.startswith(text)]

    if state < len(options):
        return options[state]
    else:
        return None

def main(serial_id):
    port = SerialPort(serial_id)
    monitor = DockerMonitor()
    global commands
    commands = commands + monitor.get_tags()

    signal.signal(signal.SIGINT, signal_handler)
    print('Welcome to TX')
    print_help()


    readline.parse_and_bind("tab:complete")
    readline.set_completer(completer)
    while True:
        user_input = input('#: ')
        tokens = user_input.split()

        if len(tokens) == 0:
            continue

        cmd = tokens[0]
        
        if cmd == 'help':
            print_help()
        elif cmd == 'tags':
            print('Possible tags: {}'.format(tags))
        elif cmd == 'exit':
            break
        elif cmd == 'run':
            tag = 'latest'
            if len(tokens) > 1:
                tag = tokens[1]

            if not monitor.check_image(tag):
                continue
            
            port.write(cmd + ' ' + tag)

            if not port.check_ack():
                print('Did not receive ack')
                continue

            if not port.check_succeeded(tries=20):
                print('Failed to start container')
            else:
                print('Container started')

        else:
            print('Unknown command: {}'.format(cmd))

    # Close port on exit
    port.close()
    monitor.close()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Need to pass the device ID in')
        exit()

    main(sys.argv[1])
