
import serial
import time
def set_target(serial_port, channel, target):

    target_bytes = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
    serial_port.write(target_bytes)


def get_position(serial_port, channel):

    serial_port.write(bytearray([0x90, channel]))
    response = serial_port.read(2)
    position = response[0] + 256 * response[1]
    return position
def main():
    with serial.Serial('/dev/tty.usbmodem004148121', 9600) as maestro_serial:

        for target in [2000, 8000]:
            for channel in range(5):
                set_target(maestro_serial, channel, target)
                time.sleep(0.2)

                position = get_position(maestro_serial, channel)
                print(f"Current Position: {position}")



if __name__ == '__main__':
    main()