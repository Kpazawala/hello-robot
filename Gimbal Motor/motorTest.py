import serial
import time
import threading
import tty
import sys
import termios

# Define the serial port and baud rate.
ser = None
def open_serial_port():
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"Successfully opened port: {port}")
            return ser
        except serial.SerialException as e:
            print(f"Failed to open port {port}: {e}")
    raise RuntimeError("Failed to open any serial port.")

try:
    ser = open_serial_port()
    time.sleep(2)
except RuntimeError as e:
    print(f"Error: {e}")


# Define the modes for reference
modes = [
    "M1",  # velocity_control | controls: T$ target, TS# to ramp
    "M2",  # angle_control | controls: T# to ramp
    "M3",  # torque_control | controls: T# target
    "M4",  # PID_tuning | controls: PP# PI# PD# T# target
]

# Create an event to signal the read thread to stop
stop_event = threading.Event()
orig_settings = termios.tcgetattr(sys.stdin)
def read_from_serial():
    """Thread function to read data from the serial port."""
    while True:
        try:
            if ser.is_open > 0 and not stop_event.is_set():
                try:
                    msg = ser.readline().decode().strip()
                    # Print the message received
                    try:
                        target, voltage, velocity, angle = msg.split()[:4]
                        print(f"target: {target}, voltage: {voltage}, velocity: {velocity}, angle: {angle}")
                    except ValueError:
                        print("Arduino Response:", msg)
                except serial.SerialException as e:
                    print(f"Serial read error: {e}")
                    break  # Exit the loop if there’s a read error
                time.sleep(.1)  # Sleep to avoid busy-waiting
            else:
                ser.reset_input_buffer()
        except KeyboardInterrupt:
            print("Interrupted")
            break
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    ser.close()

def handle_key_press():
    """Detect key presses and manage threads accordingly."""
    x = ''
    try:
        while not stop_event.is_set():
            tty.setcbreak(sys.stdin)
            x = sys.stdin.read(1)  # Read a single character
            if x == 'w':
                print("Key 'w' pressed")

                stop_event.set()  # Signal other threads to stop
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
                a = input("Enter command: ")
                if ser.is_open:
                    ser.write(f"{a}\n".encode('utf-8'))  # Replace with the actual command
                stop_event.clear()
            

    except KeyboardInterrupt:
        print("Interrupted by user")

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)  # Restore original settings
    print("Terminal settings restored")

# Start the threads
serial_thread = threading.Thread(target=read_from_serial)
key_thread = threading.Thread(target=handle_key_press)

serial_thread.start()
key_thread.start()

try:
    serial_thread.join()
    key_thread.join()
except KeyboardInterrupt:
    # If interrupted, signal the threads to stop and wait for them to finish
    stop_event.set()
    serial_thread.join()
    key_thread.join()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
    print("All threads have ended")