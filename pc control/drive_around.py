# UPD / TCP hybrid connection demo
# handshake with TCP, data transmission with UDP
# drive around with the excavator

from control_modules import joystick_module, socket_manager
from time import sleep


addr = '10.214.33.12'

port = 5111


# Who am I. Check config for names
identification_number = 2  # 0 excavator, 1 Mevea, 2 Motion Platform, more can be added...
# My (num) inputs I want to receive
inputs = 0
# My (num) outputs im going to send
outputs = 10


# Frequency of the send/receive loop in Hz
# 20Hz ~50ms        As of 17.4.2024, the ISM330DHCX are set to 26Hz max
# 50Hz ~20ms.       PWM controlled servos live in this area
# 60Hz ~17ms        ADCpi max read speed at 14-bit
# 200Hz ~5ms etc.
loop_frequency = 20 # hz

# For saving bandwidth
# 1-byte signed int goes from -128 to 127
int_scale = 127


# init xbox controller
controller = joystick_module.XboxController()


# init socket
socket = socket_manager.MasiSocketManager()


# set up the PC as client. set connect_addr if client and TCP
if not socket.setup_socket(addr, port, identification_number, inputs, outputs, socket_type='client'):
    raise Exception("could not setup socket!")

# setup done

# when not communicating with Mevea, you are able to send three extra arguments
# example: send used loop_frequency for safety, int_scale for bandwidth saving and local_datatype for data type (we are sending integers)
handshake_result, extra_args = socket.handshake(extra_arg1=loop_frequency, extra_arg2=int_scale, local_datatype='int')

if not handshake_result:
    raise Exception("could not make handshake!")

# switcheroo
socket.tcp_to_udp()

sleep(5)


def float_to_int(data, scale=int_scale):  # using 1-byte unsigned int, mapped -1 to +1. A bit ghetto but basically works with dinosaur-era networking
    int_data = []  # List to store converted integer values

    for value in data:
        clamped_value = max(-1.0, min(1.0, value))
        int_value = int(clamped_value * scale)
        int_data.append(int_value)

    return int_data


def run():
    while True:
        # read controller values
        joy_values = controller.read()

        if joy_values['RightBumper']:
            joy_values['RightTrigger'] = -joy_values['RightTrigger']


        if joy_values['LeftBumper']:
            joy_values['LeftTrigger'] = -joy_values['LeftTrigger']


        controller_list = [
            joy_values['RightJoystickX'], # scoop, index 0
            joy_values['LeftJoystickY'],  # tilt boom, index 1
            joy_values['LeftJoystickX'],  # rotate cabin, index 2
            joy_values['RightJoystickY'],  # lift boom, index 3
            joy_values['RightTrigger'],    # track R, index 4
            joy_values['LeftTrigger'],   # track L, index 5
            joy_values['A'],   # index 6
            joy_values['B'],   # index 7
            joy_values['X'],   # index 8
            joy_values['Y'],   # index 9
            ]


        # to save bandwidth, convert -1...1 floating point numbers (4-8 bytes) to -100...100 ints (1 byte)
        # we do the reverse on the receiving end
        # this is a simple way to save bandwidth, but it limits the precision of the values
        int_joystick_data = float_to_int(controller_list)

        socket.send_data(int_joystick_data)
        print(f"Sent: {int_joystick_data}")


        sleep(1/loop_frequency) # rough polling rate



if __name__ == "__main__":
    run()
