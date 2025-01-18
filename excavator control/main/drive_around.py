import asyncio
import time
from control_modules import PWM_controller, socket_manager #ADC_sensors #IMU_sensors

addr = '10.214.33.12'
port = 5111

identification_number = 0  # 0 excavator, 1 Mevea, 2 Motion Platform, more can be added...
inputs = 10  # Number of inputs received from the other end
outputs = 0  # Number of outputs I'm going to send.

# Initialize PWM controller
pwm = PWM_controller.PWM_hat(
    config_file='configuration_files/excavator_channel_configs.yaml',
    simulation_mode=False,
    pump_variable=True,
    tracks_disabled=True,
    deadzone=0.8,
    input_rate_threshold=5
)

# Initialize socket
socket = socket_manager.MasiSocketManager()

# Set up Excavator as server
if not socket.setup_socket(addr, port, identification_number, inputs, outputs, socket_type='server'):
    raise Exception("Could not set up socket!")

# Receive handshake and extra arguments
handshake_result, extra_args = socket.handshake(example_arg_you_could_send_here=69)

if not handshake_result:
    raise Exception("Could not make handshake!")

loop_frequency, int_scale = extra_args[0], extra_args[1]
print(f"Received extra arguments: Loop_freq ({loop_frequency}), int_scale ({int_scale})")

# Update the PWM-controller safety threshold to be 10% of the loop_frequency
pwm.set_threshold(loop_frequency * 0.10)

# Switch socket communication to UDP (to save bandwidth)
socket.tcp_to_udp()



def int_to_float(int_data, decimals=2, scale=int_scale):
    return [round((value / scale), decimals) for value in int_data]

class ButtonState:
    # Button state tracking
    def __init__(self):
        self.states = {}

    def check_button(self, button_index, value, threshold=0.8):
        # check rising edge. Return True if button is pressed and was not pressed before
        # threshold 0.8 because we mangled the data with int_scale
        current_state = value > threshold
        previous_state = self.states.get(button_index, False)
        self.states[button_index] = current_state
        return current_state and not previous_state

async def async_main():
    control_task = asyncio.create_task(control_signal_loop(20))
    # add more non-blocking tasks here (e.g. sensor reading and sending)
    await asyncio.gather(control_task) # remember to add task names here as well

async def control_signal_loop(frequency):
    interval = 1.0 / frequency
    step = 0
    button_state = ButtonState()

    while True:
        start_time = time.time()

        value_list = socket.get_latest_received()

        if value_list is not None:
            float_values = int_to_float(value_list)

            control_values = float_values[:6]
            # rest of the values we can use here

            print(f"control values: {control_values})
            #pwm.update_values(control_values)

            # print average input rate roughly every second
            if step % loop_frequency == 0:
                print(f"Avg rate: {pwm.get_average_input_rate():.2f} Hz")
            step += 1


            # Button checks (mostly placeholders)
            if button_state.check_button(8, float_values[8]):
                print("button 8 pressed")


            if button_state.check_button(9, float_values[9]):
                print("button 9 pressed")



        elapsed_time = time.time() - start_time
        await asyncio.sleep(max(0, interval - elapsed_time))

def run():
    socket.start_data_recv_thread()
    asyncio.run(async_main())

if __name__ == "__main__":
    try:
        run()
    finally:
        pwm.reset()
        socket.stop_all()
        pwm.stop_monitoring()