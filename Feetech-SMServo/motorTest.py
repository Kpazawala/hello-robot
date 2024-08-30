#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SMServo model on this example : All models using Protocol SMS
# This example is tested with a SMServo(STS/SMS/SMS), and an URT
# Be sure that SMServo(STS/SMS/SMS) properties are already set as %% ID : 1 / Baudnum : 6 (Baudrate : 1000000)
#
#Uses Feetech Servo SDK Library https://gitee.com/ftservo/SCServoSDK

import os
import threading
import csv

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from scservo_sdk import *                    # Uses SMServo SDK library

# Control table address
ADDR_SMS_TORQUE_ENABLE      = 40
ADDR_SMS_GOAL_ACC           = 41
ADDR_SMS_GOAL_POSITION      = 42
ADDR_SMS_GOAL_VEL           = 46
ADDR_SMS_PRESENT_POSITION   = 56
ADDR_SMS_PRESENT_IN_VOLTAGE = 62
ADDR_SMS_PRESENT_CURRENT    = 69
ADDR_SMS_PRESENT_TEMPERATURE= 63
ADDR_SMS_WORK_MODE          = 33

ADDR_SMS_OVERLORD_CURRENT   = 28
ADDR_SMS_OVERLORD_TORQUE    = 36

# Default setting
SMS_ID                      = 2                # SMServo ID : 1
BAUDRATE                    = 115200            # SMServo default baudrate : 1000000 | SMServo SMS default baudrate : 115200
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


SMS_WORK_MODE_VEL = 1
SMS_WORK_MODE_POS = 0
SMS_MINIMUM_POSITION_VALUE  = 0         # SMServo will rotate between this value
SMS_MAXIMUM_POSITION_VALUE  = 4093        # and this value (note that the SMServo would not move when the position value is out of movable range. Check e-manual about the range of the SMServo you use.)
SMS_MOVING_STATUS_THRESHOLD = 10          # SMServo moving status threshold
SMS_MOVING_VEL            = 0          # SMServo moving speed
SMS_MOVING_ACC              = 0          # SMServo moving acc
protocol_end                = 0           # SMServo bit end(STS/SMS=0, SMS=1)

MODE = 1
mode_set, target_set = False, False
min_max_oscillate = []

index = 0
sms_goal_position = 0         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = PacketHandler(protocol_end)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def error_handler(sms_comm_result, sms_error, sms_model_number = None):

    if sms_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sms_comm_result))
    elif sms_error != 0:
        print("%s" % packetHandler.getRxPacketError(sms_error))
    elif sms_model_number != None:
        print("[ID:%03d] ping Succeeded. SMServo model number : %d" % (SMS_ID, sms_model_number))

# Try to ping the SMServo
# Get SMServo model number
sms_model_number, sms_comm_result, sms_error = packetHandler.ping(portHandler, SMS_ID)
error_handler(sms_comm_result, sms_error, sms_model_number)

sms_overload_torque, sms_comm_result, sms_error = packetHandler.read1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_OVERLORD_TORQUE)
error_handler(sms_comm_result, sms_error)

sms_overload_current, sms_comm_result, sms_error = packetHandler.read1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_OVERLORD_CURRENT)
error_handler(sms_comm_result, sms_error)

print(f"Overload Torque: {sms_overload_torque} Overload Current: {sms_overload_current}")

def handle_command(user_command):
    global MODE, mode_set, target_set, sms_goal_position, min_max_oscillate
    cmd = user_command[:2]
    sub_cmd = user_command[2:].strip().split()
    MODE = int(user_command[1]) if user_command[0] == 'm' else MODE

    sms_comm_result = None
    sms_error = None

    # Define the modes for reference
    modes = [
        "m1",  # velocity_control | controls: T$ target, TS# to ramp
        "m2",  # angle_control | controls: T# to ramp
        "m3",  # oscillate
    ]
    commands = [
        "tt", #targets
        "PP", 
        "PI",
        "PD"
    ]
    match cmd:
        case "m1":
            sms_comm_result, sms_error = packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_WORK_MODE, SMS_WORK_MODE_VEL)
        case "m2":
            sms_comm_result, sms_error = packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_WORK_MODE, SMS_WORK_MODE_POS)
        case "m3":
            sms_comm_result, sms_error = packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_WORK_MODE, SMS_WORK_MODE_POS)
        case "tt":
            if sub_cmd:
                sms_goal_position = 0
                min_max_oscillate = []
                match MODE:
                    case 1: 
                        if len(sub_cmd) == 2:
                            SMS_MOVING_VEL, SMS_MOVING_ACC = int(sub_cmd[0]), int(sub_cmd[1])
                        else:
                            SMS_MOVING_VEL, SMS_MOVING_ACC = int(sub_cmd[0]), 0

                        sms_comm_result, sms_error = packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_ACC, SMS_MOVING_ACC)
                        sms_comm_result, sms_error = packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_VEL, SMS_MOVING_VEL)

                    case 2: 

                        if 0 <= int(sub_cmd[0]) <= 360:
                            sms_goal_position = ( int(sub_cmd[0])/360 * SMS_MAXIMUM_POSITION_VALUE ) 
                        elif int(sub_cmd[0]) > 360:
                            sms_goal_position = SMS_MAXIMUM_POSITION_VALUE
                        else:
                            sms_goal_position = SMS_MINIMUM_POSITION_VALUE
                        sms_goal_position = int(sms_goal_position)

                        if len(sub_cmd) == 2:
                            SMS_MOVING_VEL = int(sub_cmd[1])
                        else:
                            SMS_MOVING_VEL = 100
                        
                        sms_comm_result, sms_error = packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_ACC, 0)
                        sms_comm_result, sms_error = packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_VEL, SMS_MOVING_VEL)
                        sms_comm_result, sms_error = packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_POSITION, sms_goal_position)
                    case 3: 
                        if len(sub_cmd) == 3:
                            SMS_MOVING_VEL = int(sub_cmd[2])
                        else:
                            SMS_MOVING_VEL = 100

                        c = SMS_MAXIMUM_POSITION_VALUE/360
                        min_max_oscillate = [int(int(sub_cmd[0]) * c), int(int(sub_cmd[1]) * c)]
                        sms_comm_result, sms_error = packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_ACC, 0)
                        sms_comm_result, sms_error = packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_VEL, SMS_MOVING_VEL)
                    case _:
                        print("Set Mode First")
            mode_set = True
            target_set = True
        case _:
                print("Invalid Command") 
                return
    error_handler(sms_comm_result, sms_error)

data = []
stop_event = threading.Event()
end_event = threading.Event()
def read_from_serial():
    """Thread function to read data from the serial port."""
    global mode_set, target_set
    while not mode_set and not target_set:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        command = input("Enter command: ")
        handle_command(command)
    print("ready")

  
    
    sms_error = None
    index = 0
    delay = 0
    cycles = 0
    while True:
        if not stop_event.is_set():
            if len(min_max_oscillate) != 0:
                sms_comm_result, sms_error = packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_POSITION, min_max_oscillate[index])
                error_handler(sms_comm_result, sms_error)
                
            # Read SMServo present position
            sms_present_position_velocity, sms_comm_result, sms_error = packetHandler.read4ByteTxRx(portHandler, SMS_ID, ADDR_SMS_PRESENT_POSITION)
            error_handler(sms_comm_result, sms_error)

            sms_present_position = SCS_LOWORD(sms_present_position_velocity)
            sms_present_velocity = SCS_HIWORD(sms_present_position_velocity)

            # Read SMServo present voltage and temperature
            sms_present_voltage_temp, sms_comm_result, sms_error = packetHandler.read2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_PRESENT_IN_VOLTAGE)
            error_handler(sms_comm_result, sms_error)

            sms_present_voltage = SCS_LOBYTE(sms_present_voltage_temp)
            sms_present_temp = SCS_HIBYTE(sms_present_voltage_temp)

            # Read SMServo present current
            sms_present_current, sms_comm_result, sms_error = packetHandler.read1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_PRESENT_CURRENT)
            error_handler(sms_comm_result, sms_error)



            c =  360/SMS_MAXIMUM_POSITION_VALUE
            print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d"
                % (SMS_ID, sms_goal_position * c, sms_present_position * c, SCS_TOHOST(sms_present_velocity, 15)))
            print("[ID:%03d] PresVolt:%03d PresCurent:%03d PresTemp:%03d"
                % (SMS_ID, sms_present_voltage, sms_present_current, sms_present_temp))
            
            d = [
            SMS_ID, sms_goal_position * c, sms_present_position * c,
            SCS_TOHOST(sms_present_velocity, 15),
            sms_present_voltage, sms_present_current, sms_present_temp
            ]
            d = ['%.2f' % elem for elem in d]
            data.append(d)


            if len(min_max_oscillate) != 0 and (abs(min_max_oscillate[index] * c - sms_present_position * c) < SMS_MOVING_STATUS_THRESHOLD):
                if delay > 25:
                    if index == 0:
                        index = 1
                    else:
                        index = 0 
                    delay = 0
                    cycles += 1
                else:
                    delay += 1
                print(index)

            if cycles == 10:
                end_event.set()

            if end_event.is_set() or sms_error != 0:

                packetHandler.write1ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_ACC, 0)
                packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_VEL, 0)
                sms_comm_result, sms_error = packetHandler.write2ByteTxRx(portHandler, SMS_ID, ADDR_SMS_GOAL_POSITION, sms_present_position)
                break

            


        

def handle_key_press():
    """Detect key presses and manage threads accordingly."""
    global mode_set, target_set
    while not stop_event.is_set():
        if mode_set and target_set:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            tty.setcbreak(sys.stdin)
            x = sys.stdin.read(1)  # Read a single character
            if x == 'w':
                print("Key 'w' pressed")
                stop_event.set()  # Signal other threads to stop
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                command = input("Enter command: ")
                handle_command(command)
                stop_event.clear()
            elif x == chr(0x1b):
                end_event.set()
                break
        
        
        

# Start the threads
serial_thread = threading.Thread(target=read_from_serial)
key_thread = threading.Thread(target=handle_key_press)

serial_thread.start()
key_thread.start()

try:
    serial_thread.join()
    key_thread.join()

    portHandler.closePort()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  


except KeyboardInterrupt:
    # Close port
    end_event.set()

    serial_thread.join()
    key_thread.join()
    
    portHandler.closePort()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)   
    
filename = "output.csv"
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # Write the header row
    writer.writerow(["ID", "GoalPos", "PresPos", "PresSpd", "PresVolt", "PresCurent", "PresTemp"])
    
    # Write all the data rows
    writer.writerows(data)

print(f"All data has been written to {filename}.")